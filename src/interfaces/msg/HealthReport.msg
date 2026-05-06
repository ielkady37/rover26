float32 cpu_percent           # 0-100 %
float32 ram_percent           # 0-100 %
float32 cpu_temp_celsius      # degrees Celsius (-1 if unavailable)
float32 wifi_latency_ms       # avg ping RTT to default gateway in ms  (-1 if unavailable)
float32 wifi_bandwidth_mbps   # Mbps (-1 if unavailable)
float32 disk_percent          # root partition 0-100 %
float32 battery_percent       # 0-100 % (-1 if no battery)
bool    battery_plugged        # true if charging / on AC power
string  gateway_ip             # default gateway that was pinged (e.g. 192.168.1.1)
string[] health_checks        # "ServiceName:ok" | "ServiceName:error:<reason>"
builtin_interfaces/Time stamp
