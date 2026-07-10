#!/usr/bin/env python3
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from utils.Logger import RoverLogger
from utils.Configurator import Configurator
from interfaces.msg import PotholesDetectionResult
from cv.services.PotholesDetector import PotholesDetector
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn

_QOS_DEPTH = 10


class PotholesDetectionNode(LifecycleNode):
    """Subscribes to a raw camera topic, runs target detection, and publishes results.

    Topics
    ------
    Subscribed:  /{camera_name}/uncalibrated  (sensor_msgs/Image)
    Published:   /pothole_detection           (interfaces/PotholesDetectionResult)
    """

    def __init__(self) -> None:
        super().__init__('pothole_detection_node')
        self._log = RoverLogger()
        self._bridge = CvBridge()
        self._detector = None
        self._cam_name = ''
        self._cam_sub = None
        self._result_pub = None

    # ------------------------------------------------------------------
    # Lifecycle hooks

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info('PotholesDetectionNode: configuring...')
        try:
            conf = Configurator()
            pd_cfg = conf.fetchData(Configurator.POTHOLES_DETECTION)
            if not pd_cfg:
                self._log.err('PotholesDetectionNode: potholes_detection.yaml is empty or missing.')
                return TransitionCallbackReturn.FAILURE

            cam_name = str(pd_cfg.get('camera_name', 'fisheye_camera'))
            cam_cfg = {}
            if bool(pd_cfg.get('use_fisheye', False)):
                cameras_cfg = conf.fetchData(Configurator.CAMERAS)
                cam_cfg = (cameras_cfg or {}).get(cam_name, {})
                if not cam_cfg:
                    self._log.warn(
                        f'PotholesDetectionNode: camera "{cam_name}" not found in cameras.yaml'
                        ' — fisheye remap will use config defaults.'
                    )

            self._cam_name = cam_name
            self._detector = PotholesDetector(pd_cfg, cam_cfg)
            self._detector.load()

            self._log.succ('PotholesDetectionNode: detector configured successfully.')
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self._log.err(f'PotholesDetectionNode on_configure failed: {exc}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info('PotholesDetectionNode: activating...')
        try:
            image_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=_QOS_DEPTH,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            result_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=_QOS_DEPTH,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self._cam_sub = self.create_subscription(
                Image,
                f'/{self._cam_name}/calibrated',
                self._on_frame,
                image_qos,
            )
            self._result_pub = self.create_publisher(
                PotholesDetectionResult, '/pothole_detection', result_qos
            )
            self._log.info(
                f'PotholesDetectionNode: subscribed to /{self._cam_name}/calibrated'
            )
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self._log.err(f'PotholesDetectionNode on_activate failed: {exc}')
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info('PotholesDetectionNode: deactivating...')
        if self._cam_sub is not None:
            self.destroy_subscription(self._cam_sub)
            self._cam_sub = None
        if self._result_pub is not None:
            self.destroy_publisher(self._result_pub)
            self._result_pub = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info('PotholesDetectionNode: cleaning up...')
        self._detector = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info('PotholesDetectionNode: shutting down...')
        self.on_deactivate(state)
        self.on_cleanup(state)
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Frame callback

    def _on_frame(self, msg: Image) -> None:
        try:
            raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self._log.err(f'PotholesDetectionNode: image conversion failed: {exc}')
            return

        try:
            annotated, detected, xs, ys, radii = self._detector.detect(raw)
        except Exception as exc:
            self._log.err(f'PotholesDetectionNode: detect() failed: {exc}')
            return

        try:
            result = PotholesDetectionResult()
            result.header.stamp = self.get_clock().now().to_msg()
            result.header.frame_id = self._cam_name
            result.pothole_detected = bool(detected)
            result.pothole_xs = [float(x) for x in xs]
            result.pothole_ys = [float(y) for y in ys]
            result.pothole_radii = [float(r) for r in radii]
            result.annotated_frame = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            if self._result_pub is not None:
                self._result_pub.publish(result)
        except Exception as exc:
            self._log.err(f'PotholesDetectionNode: publish failed: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PotholesDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
