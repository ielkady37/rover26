#!/usr/bin/env python3
import os
import threading
import rclpy
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from utils.Logger import RoverLogger
from utils.Configurator import Configurator
from interfaces.msg import FaceRecognitionResult
from cv.services.FaceRecognizer import FaceRecognizer
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn

_LOCK_FRAMES = 5
_QOS_DEPTH = 10
_TARGET_TOPIC = '/face_recognition/target_select'
_TARGETS_DIR = 'targets'
_SUPPORTED_TARGET_EXTS = ('.png', '.jpg', '.jpeg')

class FaceRecognitionNode(LifecycleNode):
    """Subscribes to a raw camera topic, runs face/target recognition, and publishes results.

    Topics
    ------
    Subscribed:  /{camera_name}/uncalibrated  (sensor_msgs/Image)
    Subscribed:  /face_recognition/target_select (std_msgs/String)
    Published:   /face_recognition            (interfaces/FaceRecognitionResult)
    """

    def __init__(self) -> None:
        super().__init__('face_recognition_node')
        self._log = RoverLogger()
        self._bridge = CvBridge()
        self._recognizer = None
        self._cam_name = ''
        self._targets_dir = ''
        self._cam_sub = None
        self._target_sub = None
        self._result_pub = None
        self._confirm_counter: int = 0
        self._current_target_name: str = ''
        self._recognizer_lock = threading.Lock()

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
            self._targets_dir = os.path.join(Configurator.getProjectRoot(), _TARGETS_DIR)
            target_img_path = str(fr_cfg.get('target_img_path', ''))

            loaded_name = os.path.basename(target_img_path).strip()
            self._current_target_name = os.path.splitext(loaded_name)[0] if loaded_name else ''

            with self._recognizer_lock:
                self._recognizer = None

            # Loading YOLO + OSNet and building the target embedding can take
            # several seconds (longer on first CUDA init) — doing that inline
            # would stall this ChangeState response long enough for the
            # middleware to drop it before MissionManagerNode ever sees it.
            # Build/load in the background; on_activate/_on_frame guard on
            # recognizer readiness in the meantime.
            threading.Thread(
                target=self._load_recognizer_async,
                args=(fr_cfg, cam_cfg, target_img_path),
                daemon=True,
                name='face_recognizer_load',
            ).start()

            self._log.info('FaceRecognitionNode: model loading dispatched in background.')
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self._log.err(f'FaceRecognitionNode on_configure failed: {exc}')
            return TransitionCallbackReturn.FAILURE

    def _load_recognizer_async(self, fr_cfg: dict, cam_cfg: dict, target_img_path: str) -> None:
        try:
            recognizer = FaceRecognizer(fr_cfg, cam_cfg)
            recognizer.load(target_img_path)
            with self._recognizer_lock:
                self._recognizer = recognizer
            self._log.succ('FaceRecognitionNode: model loaded successfully.')
        except Exception as exc:
            self._log.err(f'FaceRecognitionNode: background model load failed: {exc}')

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
            self._target_sub = self.create_subscription(
                String,
                _TARGET_TOPIC,
                self._on_target_selected,
                result_qos,
            )
            self._result_pub = self.create_publisher(
                FaceRecognitionResult, '/face_recognition', result_qos
            )
            self._confirm_counter = 0
            self._log.info(
                f'FaceRecognitionNode: subscribed to /{self._cam_name}/calibrated'
            )
            self._log.info(
                f'FaceRecognitionNode: waiting for target selections on {_TARGET_TOPIC}'
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
        if self._target_sub is not None:
            self.destroy_subscription(self._target_sub)
            self._target_sub = None
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
    # Target selection callback

    def _on_target_selected(self, msg: String) -> None:
        target_name = str(msg.data).strip()
        if not target_name:
            self._log.warn('FaceRecognitionNode: received empty target name.')
            return

        if self._recognizer is None:
            self._log.warn('FaceRecognitionNode: recognizer is not ready yet.')
            return

        if target_name == self._current_target_name:
            return

        try:
            target_path = self._resolve_target_path(target_name)
        except Exception as exc:
            self._log.err(f'FaceRecognitionNode: invalid target "{target_name}": {exc}')
            return

        try:
            with self._recognizer_lock:
                self._recognizer.load(target_path)
            self._confirm_counter = 0
            self._current_target_name = os.path.splitext(os.path.basename(target_path))[0]
            self._log.succ(
                f'FaceRecognitionNode: target updated to {self._current_target_name} ({target_path})'
            )
        except Exception as exc:
            self._log.err(f'FaceRecognitionNode: failed to load target "{target_name}": {exc}')

    def _resolve_target_path(self, target_name: str) -> str:
        if not self._targets_dir or not os.path.isdir(self._targets_dir):
            raise RuntimeError(f'targets directory not found: {self._targets_dir}')

        base_name, given_ext = os.path.splitext(target_name)
        candidate_names = []

        if given_ext:
            candidate_names.append(target_name)
        else:
            for ext in _SUPPORTED_TARGET_EXTS:
                candidate_names.append(f'{target_name}{ext}')

        lowered = {name.lower() for name in candidate_names}
        for entry in os.listdir(self._targets_dir):
            path = os.path.join(self._targets_dir, entry)
            if not os.path.isfile(path):
                continue

            entry_stem, entry_ext = os.path.splitext(entry)
            if entry_ext.lower() not in _SUPPORTED_TARGET_EXTS:
                continue

            if entry.lower() in lowered:
                return path

            if not given_ext and entry_stem.lower() == target_name.lower():
                return path

        tried = ', '.join(candidate_names)
        raise FileNotFoundError(f'no target image found for "{target_name}" (tried: {tried})')

    # ------------------------------------------------------------------
    # Frame callback

    def _on_frame(self, msg: Image) -> None:
        with self._recognizer_lock:
            recognizer = self._recognizer
        if recognizer is None:
            return  # model still loading in the background, or configure failed

        try:
            raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self._log.err(f'FaceRecognitionNode: image conversion failed: {exc}')
            return

        try:
            with self._recognizer_lock:
                annotated, is_detected, score = recognizer.detect(raw)
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
