#!/usr/bin/env python3
import os
import cv2
import torch
import numpy as np
from PIL import Image
from ultralytics import YOLO
from img2vec_pytorch import Img2Vec
from utils.Configurator import Configurator
from cv.helpers.CVUtilities import CVUtilities
from sklearn.metrics.pairwise import cosine_similarity

class FaceRecognizer:
    """Target face/person recognition using YOLOv8 detection + Img2Vec similarity.

    Parameters
    ----------
    fr_config:
        Dict loaded from ``face_recognition.yaml``.  Required keys:
        ``yolo_model_path``, ``similarity_threshold``, ``use_gpu``, ``balance``.
    camera_config:
        Dict loaded from ``cameras.yaml`` for the subscribed camera.  Used to
        obtain ``width``, ``height``, and ``calibration`` path for building
        the fisheye undistort maps.
    """

    def __init__(self, fr_config: dict, camera_config: dict) -> None:
        self._threshold = float(fr_config.get('similarity_threshold', 0.70))
        self._balance = float(fr_config.get('balance', 0.0))
        self._use_gpu = bool(fr_config.get('use_gpu', False))
        self._model_path = str(fr_config.get('yolo_model_path', ''))

        self._width = int(camera_config.get('width', 320))
        self._height = int(camera_config.get('height', 240))
        self._calib_path = camera_config.get('calibration')

        self._model = None
        self._img2vec = None
        self._target_vector = None
        self._map1 = None
        self._map2 = None
        self._ready = False

    # ------------------------------------------------------------------
    # Lifecycle

    def load(self, target_img_path: str) -> None:
        """Load the YOLO model, configure the device, build undistort maps,
        and extract the target reference vector.

        Raises
        ------
        RuntimeError
            When the target image is missing or any component fails to load.
        """
        try:

            root = Configurator.getProjectRoot()
            model_abs = self._resolve_path(self._model_path, root)
            target_abs = self._resolve_path(target_img_path, root)

            use_cuda = self._use_gpu and torch.cuda.is_available()
            device = 'cuda' if use_cuda else 'cpu'

            self._model = YOLO(model_abs)
            self._model.to(device)

            self._img2vec = Img2Vec(cuda=use_cuda)

            target_img = cv2.imread(target_abs)
            if target_img is None:
                raise RuntimeError(f'Target image not found: {target_abs}')
            target_pil = Image.fromarray(cv2.cvtColor(target_img, cv2.COLOR_BGR2RGB))
            self._target_vector = self._img2vec.get_vec(target_pil).reshape(1, -1)

            if self._calib_path:
                calib_abs = self._resolve_path(str(self._calib_path), root)
                self._map1, self._map2 = CVUtilities.build_maps(
                    calib_abs, self._balance, self._width, self._height
                )

            self._ready = True
        except Exception as exc:
            self._ready = False
            raise RuntimeError(f'FaceRecognizer.load failed: {exc}') from exc

    # ------------------------------------------------------------------
    # Detection

    def detect(self, raw_frame: np.ndarray):
        """Run detection on one raw (uncalibrated) frame.

        Applies fisheye remap when calibration maps are available, then
        runs YOLO person detection and compares each crop against the target
        vector via cosine similarity.

        Parameters
        ----------
        raw_frame:
            BGR image array from the ROS uncalibrated topic.

        Returns
        -------
        tuple[np.ndarray, bool, float]
            ``(annotated_frame, is_detected, best_score)``.
            *annotated_frame* has a green bounding box drawn when detected.
            Both ``is_detected`` and ``best_score`` are ``False``/``0.0`` on
            any internal failure.
        """
        if not self._ready or raw_frame is None:
            return raw_frame, False, 0.0

        try:
            # frame = (
            #     cv2.remap(raw_frame, self._map1, self._map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            #     if self._map1 is not None
            #     else raw_frame.copy()
            # )
            frame = raw_frame.copy()
            results = self._model(frame, stream=True, verbose=False)
            best_score = 0.0
            best_box = None

            for r in results:
                for box in r.boxes:
                    if int(box.cls[0]) != 0:   # class 0 = person
                        continue

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    x1c = max(0, x1)
                    y1c = max(0, y1)
                    x2c = min(self._width, x2)
                    y2c = min(self._height, y2)

                    crop = frame[y1c:y2c, x1c:x2c]
                    if crop.size == 0 or crop.shape[0] < 20 or crop.shape[1] < 10:
                        continue

                    try:
                        person_pil = Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
                        person_vec = self._img2vec.get_vec(person_pil).reshape(1, -1)
                        score = float(cosine_similarity(self._target_vector, person_vec)[0][0])
                        if score > best_score:
                            best_score = score
                            best_box = (x1, y1, x2, y2)
                    except Exception:
                        continue

            is_detected = best_box is not None and best_score >= self._threshold
            if is_detected:
                bx1, by1, bx2, by2 = best_box
                cv2.rectangle(frame, (bx1, by1), (bx2, by2), (0, 255, 0), 4)
                cv2.putText(
                    frame, f'TARGET {best_score:.2f}',
                    (bx1, by1 - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
                )

            return frame, is_detected, best_score
        except Exception:
            return raw_frame, False, 0.0

    # ------------------------------------------------------------------
    # Internals

    @staticmethod
    def _resolve_path(path: str, root: str) -> str:
        """Resolve a repo-relative path (starting with /) against the project root."""
        if os.path.isabs(path) and not os.path.exists(path):
            return os.path.join(root, path.lstrip('/'))
        return path
