#!/usr/bin/env python3
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from utils.Logger import RoverLogger
from utils.Configurator import Configurator
from interfaces.msg import FaceRecognitionResult
from cv.services.FaceRecognizer import FaceRecognizer
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn

_LOCK_FRAMES = 5
_QOS_DEPTH = 10

class FaceRecognitionNode(LifecycleNode):
    """Subscribes to a raw camera topic, runs face/target recognition, and publishes results.

    Topics
    ------
    Subscribed:  /{camera_name}/uncalibrated  (sensor_msgs/Image)
    Published:   /face_recognition            (interfaces/FaceRecognitionResult)
    """

    def __init__(self) -> None:
        super().__init__('face_recognition_node')
        self._log = RoverLogger()
        self._bridge = CvBridge()
        self._recognizer = None
        self._cam_name = ''
        self._cam_sub = None
        self._result_pub = None
        self._confirm_counter: int = 0

    # ------------------------------------------------------------------
    # Lifecycle hooks

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info('FaceRecognitionNode: configuring...')
        try:
            conf = Configurator()
            fr_cfg = conf.fetchData(Configurator.FACE_RECOGNITION)
            if not fr_cfg:
                self._log.err('FaceRecognitionNode: face_recognition.yaml is empty or missing.')
                return TransitionCallbackReturn.FAILURE

            cam_name = str(fr_cfg.get('camera_name', 'fisheye_camera'))
            cameras_cfg = conf.fetchData(Configurator.CAMERAS)
            cam_cfg = (cameras_cfg or {}).get(cam_name, {})
            if not cam_cfg:
                self._log.warn(
                    f'FaceRecognitionNode: camera "{cam_name}" not found in cameras.yaml'
                    ' — using defaults.'
                )

            self._cam_name = cam_name
            self._recognizer = FaceRecognizer(fr_cfg, cam_cfg)
            target_img_path = str(fr_cfg.get('target_img_path', ''))
            self._recognizer.load(target_img_path)

            self._log.succ('FaceRecognitionNode: model loaded successfully.')
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self._log.err(f'FaceRecognitionNode on_configure failed: {exc}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info('FaceRecognitionNode: activating...')
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
                FaceRecognitionResult, '/face_recognition', result_qos
            )
            self._confirm_counter = 0
            self._log.info(
                f'FaceRecognitionNode: subscribed to /{self._cam_name}/calibrated'
            )
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self._log.err(f'FaceRecognitionNode on_activate failed: {exc}')
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info('FaceRecognitionNode: deactivating...')
        if self._cam_sub is not None:
            self.destroy_subscription(self._cam_sub)
            self._cam_sub = None
        if self._result_pub is not None:
            self.destroy_publisher(self._result_pub)
            self._result_pub = None
        self._confirm_counter = 0
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info('FaceRecognitionNode: cleaning up...')
        self._recognizer = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info('FaceRecognitionNode: shutting down...')
        self.on_deactivate(state)
        self.on_cleanup(state)
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Frame callback

    def _on_frame(self, msg: Image) -> None:
        try:
            raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self._log.err(f'FaceRecognitionNode: image conversion failed: {exc}')
            return

        try:
            annotated, is_detected, score = self._recognizer.detect(raw)
        except Exception as exc:
            self._log.err(f'FaceRecognitionNode: detect() failed: {exc}')
            return

        if is_detected:
            self._confirm_counter += 1
        else:
            self._confirm_counter = 0

        lock_confirmed = self._confirm_counter >= _LOCK_FRAMES

        try:
            result = FaceRecognitionResult()
            result.header.stamp = self.get_clock().now().to_msg()
            result.header.frame_id = self._cam_name
            result.target_detected = bool(is_detected)
            result.confidence_score = float(score)
            result.lock_confirmed = bool(lock_confirmed)
            result.annotated_frame = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            if self._result_pub is not None:
                self._result_pub.publish(result)
        except Exception as exc:
            self._log.err(f'FaceRecognitionNode: publish failed: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FaceRecognitionNode()
    try:
        while rclpy.ok():
            try:
                rclpy.spin(node)
                break
            except KeyboardInterrupt:
                break
            except Exception as exc:
                # An invalid lifecycle request raises out of the executor and
                # would kill the process — log and keep spinning instead.
                node.get_logger().error(f'spin error (continuing): {exc}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
