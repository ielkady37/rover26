#!/usr/bin/env python3
import rclpy
import threading
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from utils.Logger import RoverLogger
from utils.Configurator import Configurator
from cv.services.CameraStreamer import CameraStreamer
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn



_RECONNECT_BASE_S = 1.0
_RECONNECT_MAX_S = 16.0
_QOS_DEPTH = 10


class CameraStreamingNode(LifecycleNode):
    """Streams calibrated and uncalibrated frames for every camera in cameras.yaml.

    Topics published per camera:
        /{camera_name}/uncalibrated  (sensor_msgs/Image)  — always
        /{camera_name}/calibrated    (sensor_msgs/Image)  — only when calibration is configured

    Each camera runs in its own daemon thread, started on lifecycle activate.
    """

    def __init__(self) -> None:
        super().__init__('camera_streaming_node')
        self._log = RoverLogger()
        self._bridge = CvBridge()
        self._streamers: dict[str, CameraStreamer] = {}
        # name → {'uncalibrated': Publisher, 'calibrated': Publisher | None}
        self._cam_pubs: dict[str, dict] = {}
        self._threads: dict[str, threading.Thread] = {}
        self._stop_events: dict[str, threading.Event] = {}

    # ------------------------------------------------------------------
    # Lifecycle hooks

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info('CameraStreamingNode: configuring...')
        try:
            cameras_cfg = Configurator().fetchData(Configurator.CAMERAS)
            if not cameras_cfg:
                self._log.warn('CameraStreamingNode: cameras.yaml is empty — nothing to stream.')
                return TransitionCallbackReturn.SUCCESS

            image_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=_QOS_DEPTH,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )

            for cam_name, cam_cfg in cameras_cfg.items():
                streamer = CameraStreamer(cam_name, cam_cfg)
                self._streamers[cam_name] = streamer

                pubs = {
                    'uncalibrated': self.create_publisher(
                        Image, f'/{cam_name}/uncalibrated', image_qos
                    ),
                    'calibrated': (
                        self.create_publisher(Image, f'/{cam_name}/calibrated', image_qos)
                        if cam_cfg.get('calibration')
                        else None
                    ),
                }
                self._cam_pubs[cam_name] = pubs

                cal_status = 'with calibration' if pubs['calibrated'] else 'no calibration'
                self._log.info(f'CameraStreamingNode: registered "{cam_name}" ({cal_status})')

            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self._log.err(f'CameraStreamingNode on_configure failed: {exc}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info('CameraStreamingNode: activating...')
        try:
            for cam_name, streamer in self._streamers.items():
                if not streamer.open():
                    self._log.warn(
                        f'CameraStreamingNode: could not open "{cam_name}" — '
                        'will retry inside the streaming thread.'
                    )

                stop_event = threading.Event()
                self._stop_events[cam_name] = stop_event

                thread = threading.Thread(
                    target=self._stream_loop,
                    args=(cam_name, streamer, self._cam_pubs[cam_name], stop_event),
                    name=f'cam_{cam_name}',
                    daemon=True,
                )
                self._threads[cam_name] = thread
                thread.start()
                self._log.info(f'CameraStreamingNode: stream thread started for "{cam_name}"')

            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self._log.err(f'CameraStreamingNode on_activate failed: {exc}')
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info('CameraStreamingNode: deactivating...')
        for event in self._stop_events.values():
            event.set()
        for cam_name, thread in self._threads.items():
            thread.join(timeout=5.0)
            if thread.is_alive():
                self._log.warn(f'CameraStreamingNode: thread for "{cam_name}" did not stop in time.')
        self._threads.clear()
        self._stop_events.clear()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info('CameraStreamingNode: cleaning up...')
        for streamer in self._streamers.values():
            streamer.release()
        self._streamers.clear()
        self._cam_pubs.clear()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info('CameraStreamingNode: shutting down...')
        self.on_deactivate(state)
        self.on_cleanup(state)
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Per-camera streaming thread

    def _stream_loop( self, name: str, streamer: CameraStreamer, pubs: dict, stop_event: threading.Event) -> None:
        """Capture and publish frames for one camera, reconnecting on failure."""
        frame_sleep_s = 1.0 / max(streamer.fps, 1)
        backoff = _RECONNECT_BASE_S

        while not stop_event.is_set():
            raw, calibrated = streamer.get_frame()

            if raw is None:
                # Capture failed — release and retry with exponential backoff
                streamer.release()
                self._log.warn(
                    f'CameraStreamingNode: "{name}" frame read failed — '
                    f'reconnecting in {backoff:.1f}s'
                )
                if stop_event.wait(timeout=backoff):
                    break  # stop requested during wait
                backoff = min(backoff * 2, _RECONNECT_MAX_S)
                if streamer.open():
                    self._log.succ(f'CameraStreamingNode: "{name}" reconnected successfully.')
                    backoff = _RECONNECT_BASE_S
                continue

            backoff = _RECONNECT_BASE_S  # reset after a successful frame

            now = self.get_clock().now().to_msg()
            try:
                msg = self._bridge.cv2_to_imgmsg(raw, encoding='bgr8')
                msg.header.stamp = now
                msg.header.frame_id = name
                pubs['uncalibrated'].publish(msg)
            except Exception as exc:
                self._log.err(f'CameraStreamingNode: uncalibrated publish error for "{name}": {exc}')

            if calibrated is not None and pubs.get('calibrated') is not None:
                try:
                    cal_msg = self._bridge.cv2_to_imgmsg(calibrated, encoding='bgr8')
                    cal_msg.header.stamp = now
                    cal_msg.header.frame_id = name
                    pubs['calibrated'].publish(cal_msg)
                except Exception as exc:
                    self._log.err(f'CameraStreamingNode: calibrated publish error for "{name}": {exc}')

            stop_event.wait(timeout=frame_sleep_s)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraStreamingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()