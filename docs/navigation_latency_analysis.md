# Navigation Latency Analysis

## System Data Flow (End-to-End)

```
Bluetooth Joystick
      │  (BT HID ~8–16 ms)
      ▼
Laptop Browser (Web GUI)
      │  rosbridge WebSocket → ROS /joystick topic
      ▼
[Raspberry Pi — ROS 2]
  JoystickNode  ──(POSIX shared memory)──►  ManualNavigationNode
                                                   │
                                          Navigation Facade
                                          (DirEvaluator → PWMMapper → PID)
                                                   │
                                          /motor_commands topic
                                                   │
                                          ESPBridgeNode (/esp_tx subscriber)
                                                   │  (I2C)
                                                   ▼
                                          Actuator ESP32
```

---

## Stage-by-Stage Latency Breakdown

### 1. Bluetooth Joystick → Laptop

| Factor | Estimated Latency | Notes |
|---|---|---|
| Bluetooth HID polling rate | **8–16 ms** | BT Classic HID polls at 60–125 Hz |
| OS input event scheduling | ~1–2 ms | Depends on host OS scheduler |

**Verdict:** Low, unavoidable hardware floor. Not a code concern.

---

### 2. Laptop Web GUI → ROS (rosbridge)

| Factor | Estimated Latency | Notes |
|---|---|---|
| WebSocket frame transmission | ~1–5 ms (LAN) | Negligible on local network |
| rosbridge JSON serialization/deserialization | ~1–3 ms | Per message overhead |
| ROS 2 DDS publish → `/joystick` topic | <1 ms | In-process |

**Verdict:** As stated, this leg is effectively negligible on a local network.

---

### 3. `/joystick` Topic → `JoystickNode` → Shared Memory

**Code path:** `JoystickNode.callback()` → `CJoystick.updateData()`

#### Issues found:

**a) Callback-driven write with a threading lock**
```python
# Joystick.py — updateData()
with self.lock:
    ...
    self.shared_memory.buf[:len(serialized_data)] = serialized_data
```
The `threading.Lock` is shared between the writer (`JoystickNode`) and the reader (`ManualNavigationNode`). If they run in separate processes (as implied by POSIX shared memory), Python's `threading.Lock` does **not** provide cross-process mutual exclusion. This means reads could get **torn/corrupted data** rather than stale data, silently producing garbage axis values with no latency signal.

**b) Pickle serialization on every callback**
```python
serialized_data = self._serialize_data(data_with_timestamp)
```
Every joystick update runs `pickle.dumps()`. For a simple dictionary of floats/ints this is fast, but it is unnecessary overhead compared to a fixed-size struct. More importantly, a failed deserialize (`pickle.UnpicklingError`) silently returns `None`, causing the navigation node to receive a zero throttle/yaw.

**c) QoS depth = 10 on `/joystick` subscriber**
```python
self.subscriber = self.create_subscription(Joystick, '/joystick', self.callback, 10)
```
With depth=10, up to 10 messages can queue up. Under brief CPU spikes on the Pi, stale joystick frames will drain sequentially, each one writing to shared memory — meaning `ManualNavigationNode` could read data that is multiple control-loop ticks old before the queue drains.

---

### 4. Shared Memory → `ManualNavigationNode` Control Loop

**Code path:** `control_loop_callback()` → `CJoystick.__getData()` → `CJoystick.getAxis()`

#### Issues found:

**a) Polling architecture — inherent one-tick lag**
```python
# ManualNavigationNode.__init__()
timer_period = 1.0 / loop_rate   # default 100 Hz = 10 ms period
self.control_timer = self.create_timer(timer_period, self.control_loop_callback)
```
The control loop polls shared memory on a fixed timer. The joystick data could arrive at any point within a 10 ms window, meaning the worst-case lag from joystick write to motor command is **one full timer period (10 ms)** even with no other delays.

**b) Retry loop inside the hot path**
```python
# Joystick.py — __getData()
retries = 5
while retries > 0:
    ...
    time.sleep(1)   # ← 1 second sleep per retry
    retries -= 1
```
If shared memory is unavailable (e.g., `JoystickNode` hasn't written yet), `__getData()` will block for up to **5 seconds** inside the timer callback. This completely stalls the control loop and ROS executor for that duration. This is the single most dangerous latency source in the codebase.

**c) Stale data — no timestamp validation**
```python
axes = self.joystick.getAxis()
throttle = axes.get("left_y_axis", 0.0)
```
`updateData()` writes a `timestamp` field into shared memory, but `getAxis()` and `isPressed()` never check it. The control loop will happily act on arbitrarily old data (e.g., if the rosbridge connection dropped) with no awareness that the data is stale.

---

### 5. Navigation Facade Computation

**Code path:** `DirEvaluator` → `PWMMapper` → `PIDController`

#### Issues found:

**a) PID `stabilize()` called with a wall-clock `dt`**
```python
current_time = time.time()
dt = current_time - self.last_time
self.last_time = current_time
...
active_yaw_effort = self.pid.stabilize(measured_value=self.latest_yaw, dt=dt)
```
`time.time()` is not monotonic and can jump on NTP corrections. If the Pi's clock is synced via NTP (common on startup), `dt` can become negative or very large, producing a PID derivative spike that causes an erroneous motor command.

**b) IMU data arrives on a separate callback**
```python
def imu_callback(self, msg: Imu):
    ...
    self.latest_yaw = ...  # updated asynchronously
```
`latest_yaw` is written by `imu_callback` and read by `control_loop_callback` without any synchronization. Python's GIL makes this safe from corruption, but the control loop may read a yaw value from a *different time step* than the one the joystick data corresponds to, introducing a subtle timing mismatch in the PID.

---

### 6. `/motor_commands` → `ESPBridgeNode` → I2C → Actuator ESP32

**Code path:** `/motor_commands` topic → (missing direct link — see below) → `/esp_tx` → `ESPBridgeNode._esp_tx_cb()` → `I2CService.send()`

#### Issues found:

**a) Extra ROS topic hop: `/motor_commands` is not `/esp_tx`**
`ManualNavigationNode` publishes to `/motor_commands` (`MotorCommands` msg). `ESPBridgeNode` subscribes to `/esp_tx` (`ActuatorCommand` msg). There must be a translator node or this wiring is not yet complete — this is an architectural gap that adds **at least one extra ROS publish/subscribe round-trip** (another ~1–3 ms DDS hop) and a potential message-type mismatch that could silently drop commands.

**b) I2C speed — default 100 kHz (standard mode)**
```python
# I2CService — no speed configuration parameter
self._bus = smbus2.SMBus(self._bus_number)
```
`smbus2` defaults to whatever the kernel driver has configured for the bus, typically **100 kHz**. Transmitting the 19-byte `ActuatorPacket` at 100 kHz takes:
```
(1 start + 7 addr + 1 R/W + 1 ACK + 8×19 data+ack bits) ≈ 1.8 ms
```
At 400 kHz Fast-mode this drops to ~0.45 ms. There is no I2C clock configuration exposed via parameters.

**c) I2C is blocking and on the ROS executor thread**
```python
def _esp_tx_cb(self, msg):
    ...
    self._i2c_ctrl.write(cmd)   # blocking smbus2 call
```
The `_esp_tx_cb` callback runs on the main ROS executor thread. A slow I2C transaction, a NACK, or a kernel retry will block **all other callbacks** (IMU, timer, etc.) for the duration. This can cascade into missed IMU samples and timer ticks.

**d) No I2C speed / retry configurability**
`I2CService.send()` has no retry on `SensorReadError`. A single dropped I2C packet means that motor command is silently lost. The node logs a warning but does not re-attempt or queue the command.

---

### 7. SPI Reader Thread → Published Sensor Topics

**Code path:** `ESPBridgeNode._reader_loop()` → `_data_lock` → `_timer_cb()` → `/imu` publish

#### Issues found:

**a) Reader thread runs as fast as possible — no rate control**
```python
def _reader_loop(self) -> None:
    while self._running:
        try:
            data = self._spi_ctrl.receive()
            ...
```
There is no `time.sleep()` or rate limiter in `_reader_loop`. The SPI bus is hammered continuously. On a Raspberry Pi at 1 MHz SPI clock, a 62-byte packet takes ~0.5 ms, so the thread achieves ~2000 reads/s. This saturates one CPU core, competing with the ROS executor for scheduler time, which can introduce **jitter** in the publish timer and IMU callback delivery.

**b) Publish timer decoupled from SPI read — data can be up to one timer period old**
The publish timer fires at `publish_rate` Hz (default 100 Hz = 10 ms). The SPI reader could have produced a fresh sample 9.9 ms before the timer fires — that sample is published as if it is current. For high-speed maneuvers, the IMU yaw used by the PID could be one full timer tick stale.

---

## Summary Table

| Stage | Max Latency | Severity | Root Cause |
|---|---|---|---|
| BT joystick → laptop | 16 ms | 🟡 Medium | Hardware floor |
| rosbridge WebSocket | ~5 ms | 🟢 Low | Negligible on LAN |
| `/joystick` QoS queue backup | up to N×10 ms | 🟡 Medium | depth=10, CPU spike |
| Shared memory retry loop | **up to 5 s** | 🔴 **Critical** | `time.sleep(1)` in hot path |
| Stale shared memory data | unbounded | 🔴 **Critical** | No timestamp validation |
| Cross-process lock unsafety | unpredictable | 🟠 High | `threading.Lock` not cross-process |
| Control loop polling jitter | up to 10 ms | 🟡 Medium | Fixed timer, no event-driven read |
| PID `dt` NTP jump | spike (1 tick) | 🟠 High | `time.time()` not monotonic |
| `/motor_commands` → `/esp_tx` gap | ~2–5 ms extra hop | 🟠 High | Missing or extra translator node |
| I2C at 100 kHz, blocking | ~1.8 ms + jitter | 🟡 Medium | Default kernel speed, no async |
| SPI reader CPU saturation | jitter | 🟡 Medium | Unbounded tight loop |
| Publish timer / SPI decoupling | up to 10 ms stale IMU | 🟡 Medium | Timer-driven publish |

---

## Recommended Fixes (Priority Order)

1. **[Critical] Remove `time.sleep(1)` from `__getData()` retry loop** — replace with an immediate `return None` after a single failed read. Never block the ROS executor.

2. **[Critical] Add stale-data detection** — check the `timestamp` field already stored in shared memory. If `time.time() - data["timestamp"] > threshold` (e.g. 200 ms), treat it as stale and publish a safe-stop.

3. **[High] Replace `threading.Lock` with `multiprocessing.Lock` backed by shared memory** — or use a simple sequence-number / double-buffer scheme to avoid cross-process lock issues entirely.

4. **[High] Resolve `/motor_commands` → `/esp_tx` wiring gap** — confirm there is a node bridging these two topics/message-types, or collapse `ManualNavigationNode` to publish directly to `/esp_tx`.

5. **[High] Use `time.monotonic()` instead of `time.time()` for PID `dt`** — immune to NTP adjustments.

6. **[Medium] Enable I2C Fast-mode (400 kHz)** via `/boot/config.txt` (`dtparam=i2c_arm_baudrate=400000`) and expose an `i2c_baudrate` parameter in `ESPBridgeNode`.

7. **[Medium] Move I2C write off the executor thread** — use a `threading.Thread` with a queue (similar to the existing SPI reader pattern) so slow I2C transactions don't block IMU callbacks.

8. **[Medium] Add a rate limiter or `time.sleep` in `_reader_loop`** to prevent CPU saturation. A 2 kHz SPI read is far beyond the 100 Hz publish rate — a 500 Hz read rate is more than sufficient and halves CPU load.

9. **[Medium] Reduce `/joystick` subscriber QoS depth to 1** with `BEST_EFFORT` reliability to always process the latest frame and avoid queue drain lag during CPU spikes.
