import time
import rclpy
import threading
from cv_bridge import CvBridge
from rclpy.lifecycle import State
from sensor_msgs.msg import Image
from utils.Configurator import Configurator
from cv.services.CameraStreamer import CameraStreamer
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

_QOS_DEPTH = 10

class CameraStreamingNode(LifecycleNode):

    def __init__(self):
        super().__init__('camera_streaming_node')
        self._bridge = CvBridge()
        self._streamers: dict[str, CameraStreamer] = {}
        self._cam_publishers: dict[str, dict] = {}
        self._cam_timers: list = []
        self._capture_threads: list[threading.Thread] = []

        # Latest frame per camera — written by capture thread, read by timer callback
        self._latest_frames: dict[str, object] = {}
        self._frame_locks: dict[str, threading.Lock] = {}

        self._running = False
        self._logger = self.get_logger()

    # ------------------------------------------------------------------
    # Lifecycle callbacks
    # ------------------------------------------------------------------

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._logger.info('Configuring camera streamer node...')
        try:
            cameras_config: dict = Configurator().fetchData(Configurator.CAMERAS)
        except Exception as e:
            self._logger.error(f'Failed to load cameras config: {e}')
            return TransitionCallbackReturn.ERROR

        if not cameras_config:
            self._logger.error('cameras.yaml is empty or could not be loaded.')
            return TransitionCallbackReturn.ERROR

        for camera_name, config in cameras_config.items():
            streamer = CameraStreamer(camera_name, config)
            self._streamers[camera_name] = streamer
            self._cam_publishers[camera_name] = self._create_publishers(
                camera_name, streamer.is_stereo, streamer.has_calibration)
            self._logger.info(
                f"Registered camera '{camera_name}' "
                f"({'stereo' if streamer.is_stereo else 'mono'})"
            )

        self._logger.info('Camera streamer node configured successfully.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._logger.info('Activating camera streamer node...')
        self._running = True
        active_cameras = []

        for name, streamer in self._streamers.items():
            self._logger.info(f"Opening camera '{name}'...")
            if streamer.open():
                self._logger.info(f"Camera '{name}' opened successfully.")
                active_cameras.append(name)

                # Per-camera frame slot and lock
                self._latest_frames[name] = None
                self._frame_locks[name] = threading.Lock()

                # Dedicated capture thread — decoupled from ROS executor
                t = threading.Thread(
                    target=self._capture_loop,
                    args=(streamer,),
                    daemon=True,
                    name=f'capture_{name}'
                )
                t.start()
                self._capture_threads.append(t)
            else:
                self._logger.error(
                    f"Camera '{name}' could not be opened after 10 s — skipping."
                )

        if not active_cameras:
            self._running = False
            self._logger.error('All cameras failed to open. Cannot activate.')
            return TransitionCallbackReturn.FAILURE

        # Timer callbacks only grab the latest frame and publish — no blocking I/O
        for name in active_cameras:
            fps = self._streamers[name]._config.get('fps', 30)
            period = 1.0 / float(fps)
            timer = self.create_timer(
                period,
                self._make_timer_callback(name, self._cam_publishers[name])
            )
            self._cam_timers.append(timer)

        self._logger.info(
            f'Camera streamer node active with cameras: {active_cameras}'
        )
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._logger.info('Deactivating camera streamer node...')

        # Signal capture threads to stop, then wait for them
        self._running = False
        for t in self._capture_threads:
            t.join(timeout=2.0)
        self._capture_threads.clear()
        for timer in self._cam_timers:
            timer.cancel()
        self._cam_timers.clear()

        for streamer in self._streamers.values():
            streamer.release()

        self._logger.info('Camera streamer node deactivated.')
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._logger.info('Cleaning up camera streamer node...')
        self._cam_publishers.clear()
        self._streamers.clear()
        self._latest_frames.clear()
        self._frame_locks.clear()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._logger.info('Shutting down camera streamer node...')
        self._running = False
        for streamer in self._streamers.values():
            streamer.release()
        self._streamers.clear()
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Capture thread
    # ------------------------------------------------------------------

    def _capture_loop(self, streamer: CameraStreamer) -> None:
        """
        Runs in a dedicated thread — reads frames as fast as the camera
        produces them and stores the latest one. The ROS timer callback
        picks it up without ever blocking on cap.read().
        """
        while self._running:
            frame_data = streamer.read()
            if frame_data is None:
                time.sleep(0.005)
                continue
            with self._frame_locks[streamer.name]:
                self._latest_frames[streamer.name] = frame_data

    # ------------------------------------------------------------------
    # Publisher creation
    # ------------------------------------------------------------------

    def _create_publishers(self, name: str, is_stereo: bool, has_calibration: bool) -> dict:
        pubs = {}
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=_QOS_DEPTH,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        if is_stereo:
            pubs['left_raw']  = self.create_publisher(Image, f'/{name}/left/uncalibrated',  image_qos)
            pubs['right_raw'] = self.create_publisher(Image, f'/{name}/right/uncalibrated', image_qos)
            if has_calibration:
                pubs['left_cal']  = self.create_publisher(Image, f'/{name}/left/calibrated',  image_qos)
                pubs['right_cal'] = self.create_publisher(Image, f'/{name}/right/calibrated', image_qos)
        else:
            pubs['raw'] = self.create_publisher(Image, f'/{name}/uncalibrated', image_qos)
            if has_calibration:
                pubs['cal'] = self.create_publisher(Image, f'/{name}/calibrated', image_qos)
        return pubs

    # ------------------------------------------------------------------
    # Timer callback factory
    # ------------------------------------------------------------------

    def _make_timer_callback(self, name: str, pubs: dict):
        def _callback():
            # Non-blocking — just grab the latest frame the capture thread wrote
            with self._frame_locks[name]:
                frame_data = self._latest_frames.get(name)

            if frame_data is None:
                return  # capture thread hasn't produced a frame yet

            streamer = self._streamers[name]
            if streamer.is_stereo:
                self._publish_stereo(frame_data, pubs, name)
            else:
                self._publish_mono(frame_data, pubs, name)

        return _callback

    def _publish_mono(self, frame_data, pubs: dict, camera_name: str) -> None:
        try:
            stamp = self.get_clock().now().to_msg()
            raw_msg = self._bridge.cv2_to_imgmsg(frame_data.raw, encoding='bgr8')
            raw_msg.header.stamp = stamp
            raw_msg.header.frame_id = camera_name
            pubs['raw'].publish(raw_msg)
            if 'cal' in pubs:
                cal_msg = self._bridge.cv2_to_imgmsg(frame_data.calibrated, encoding='bgr8')
                cal_msg.header.stamp = stamp
                cal_msg.header.frame_id = camera_name
                pubs['cal'].publish(cal_msg)
        except Exception as e:
            self._logger.error(f'Failed to publish mono frame: {e}')

    def _publish_stereo(self, frame_data, pubs: dict, camera_name: str) -> None:
        try:
            stamp = self.get_clock().now().to_msg()
            for key, img in (
                ('left_raw',  frame_data.left_raw),
                ('left_cal',  frame_data.left_calibrated),
                ('right_raw', frame_data.right_raw),
                ('right_cal', frame_data.right_calibrated),
            ):
                if key not in pubs:
                    continue
                msg = self._bridge.cv2_to_imgmsg(img, encoding='bgr8')
                msg.header.stamp = stamp
                msg.header.frame_id = camera_name
                pubs[key].publish(msg)
        except Exception as e:
            self._logger.error(f'Failed to publish stereo frame: {e}')

def main(args=None):
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

if __name__ == '__main__':
    main()