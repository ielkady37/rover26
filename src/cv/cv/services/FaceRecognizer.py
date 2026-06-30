#!/usr/bin/env python3
import os
import cv2
import torch
import torchreid
import numpy as np
from ultralytics import YOLO
from utils.Configurator import Configurator
from cv.helpers.CVUtilities import CVUtilities


class FaceRecognizer:
    """Target person recognition using YOLOv8 detection + OSNet ReID similarity.

    Parameters
    ----------
    fr_config:
        Dict loaded from ``face_recognition.yaml``. Required keys:
        ``yolo_model_path``, ``extractor_model_path``, ``similarity_threshold``,
        ``use_gpu``, ``balance``.
    camera_config:
        Dict loaded from ``cameras.yaml`` for the subscribed camera. Used to
        obtain ``width``, ``height``, and ``calibration`` path for building
        the fisheye undistort maps.
    """

    LOCK_FRAMES_REQUIRED = 5

    def __init__(self, fr_config: dict, camera_config: dict) -> None:
        self._threshold      = float(fr_config.get('similarity_threshold', 0.70))
        self._balance        = float(fr_config.get('balance', 0.0))
        self._use_gpu        = bool(fr_config.get('use_gpu', False))
        self._model_path     = str(fr_config.get('yolo_model_path', ''))
        self._extractor_path = str(fr_config.get('extractor_model_path', ''))

        self._width      = int(camera_config.get('width', 320))
        self._height     = int(camera_config.get('height', 240))
        self._calib_path = camera_config.get('calibration')

        self._model         = None
        self._extractor     = None
        self._target_vector = None   # shape (1, D), L2-normalised
        self._map1          = None
        self._map2          = None
        self._device        = 'cpu'
        self._ready         = False

        # Consecutive-frame confirmation counter (reset on any miss)
        self._confirm_counter = 0

    # ------------------------------------------------------------------
    # Lifecycle

    def load(self, target_img_path: str) -> None:
        """Load YOLO + OSNet, build undistort maps, extract target reference vector.

        Raises
        ------
        RuntimeError
            When the target image is missing or any component fails to load.
        """
        try:
            root       = Configurator.getProjectRoot()
            model_abs  = self._resolve_path(self._model_path, root)
            target_abs = self._resolve_path(target_img_path, root)

            use_cuda     = self._use_gpu and torch.cuda.is_available()
            self._device = 'cuda' if use_cuda else 'cpu'

            # --- YOLO ---
            self._model = YOLO(model_abs)
            self._model.to(self._device)

            # --- ReID extractor ---
            self._extractor = torchreid.utils.FeatureExtractor(
                model_name='osnet_ain_x1_0',
                model_path=self._extractor_path,
                device=self._device,
            )

            # --- Fisheye maps ---
            if self._calib_path:
                calib_abs = self._resolve_path(str(self._calib_path), root)
                self._map1, self._map2 = CVUtilities.build_maps(
                    calib_abs, self._balance, self._width, self._height
                )

            # --- Target reference vector ---
            target_img = cv2.imread(target_abs)
            if target_img is None:
                raise RuntimeError(f'Target image not found: {target_abs}')

            results     = self._model(target_img, verbose=False)
            target_crop = None

            for r in results:
                for box in r.boxes:
                    if int(box.cls[0]) != 0:
                        continue
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    target_crop = target_img[y1:y2, x1:x2]
                    break
                if target_crop is not None:
                    break

            if target_crop is None:
                raise RuntimeError('No person detected in target image.')

            vec = self._extractor([target_crop])[0].cpu().numpy()
            vec = vec / np.linalg.norm(vec)
            self._target_vector = vec.reshape(1, -1)

            self._confirm_counter = 0   # clear any stale state on reload
            self._ready = True

        except Exception as exc:
            self._ready = False
            raise RuntimeError(f'FaceRecognizer.load failed: {exc}') from exc

    def reset(self) -> None:
        """Reset the confirmation counter without reloading models.

        Call this when the camera feed is interrupted, the operator
        changes target, or the node restarts tracking from scratch.
        """
        self._confirm_counter = 0

    # ------------------------------------------------------------------
    # Detection

    def detect(self, raw_frame: np.ndarray):
        """Run detection on one raw (uncalibrated) frame.

        Applies fisheye remap when calibration maps are available, then runs
        YOLO person detection and compares each crop against the stored target
        vector via cosine similarity.

        A candidate must pass the similarity threshold AND a margin check
        (best − second > 0.03) for LOCK_FRAMES_REQUIRED consecutive frames
        before ``is_locked`` is returned as ``True``.  Any single miss resets
        the counter to zero.

        Parameters
        ----------
        raw_frame:
            BGR image array from the ROS uncalibrated topic.

        Returns
        -------
        tuple[np.ndarray, bool, float]
            ``(annotated_frame, is_locked, best_score)``.
            *annotated_frame* carries a green bounding box when a candidate is
            detected, and an additional ``!!! HARD LOCK !!!`` banner once the
            counter reaches ``LOCK_FRAMES_REQUIRED``.
            ``is_locked`` and ``best_score`` are ``False``/``0.0`` on failure.
        """
        if not self._ready or raw_frame is None:
            return raw_frame, False, 0.0

        try:

            frame = raw_frame.copy()

            # 2. YOLO detection + ReID scoring
            results = self._model(frame, stream=True, verbose=False)
            scores  = []   # (similarity, (x1, y1, x2, y2))

            for r in results:
                for box in r.boxes:
                    if int(box.cls[0]) != 0:
                        continue

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    x1 = max(0, x1)
                    y1 = max(0, y1)
                    x2 = min(self._width, x2)
                    y2 = min(self._height, y2)

                    crop = frame[y1:y2, x1:x2]

                    if crop.size == 0 or crop.shape[0] < 50 or crop.shape[1] < 20:
                        continue

                    try:
                        person_vec = self._extractor([crop])[0].cpu().numpy()
                        person_vec = person_vec / np.linalg.norm(person_vec)
                        person_vec = person_vec.reshape(1, -1)

                        similarity = float(
                            np.dot(self._target_vector, person_vec.T)[0][0]
                        )
                        scores.append((similarity, (x1, y1, x2, y2)))

                    except Exception:
                        continue

            # 3. Choose best candidate
            best_score  = 0.0
            best_box    = None
            candidate_ok = False

            if scores:
                scores.sort(key=lambda x: x[0], reverse=True)
                best_score, best_box = scores[0]
                second_score = scores[1][0] if len(scores) > 1 else 0.0

                margin_ok    = len(scores) == 1 or (best_score - second_score) > 0.03
                candidate_ok = best_score >= self._threshold and margin_ok

            # 4. Update confirmation counter
            if candidate_ok:
                self._confirm_counter = min(
                    self._confirm_counter + 1,
                    self.LOCK_FRAMES_REQUIRED,
                )
            else:
                self._confirm_counter = 0

            is_locked = self._confirm_counter >= self.LOCK_FRAMES_REQUIRED

            # 5. Annotate
            if candidate_ok and best_box is not None:
                bx1, by1, bx2, by2 = best_box
                cv2.rectangle(frame, (bx1, by1), (bx2, by2), (0, 255, 0), 4)
                cv2.putText(
                    frame,
                    f'TARGET {best_score:.2f}',
                    (bx1, by1 - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )

            if is_locked:
                cv2.putText(
                    frame,
                    '!!! HARD LOCK !!!',
                    (200, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.2,
                    (0, 255, 0),
                    3,
                )

            return frame, is_locked, best_score

        except Exception:
            return raw_frame, False, 0.0

    # ------------------------------------------------------------------
    # Properties

    @property
    def confirm_counter(self) -> int:
        """Current number of consecutive confirmed frames (0 – LOCK_FRAMES_REQUIRED)."""
        return self._confirm_counter

    # ------------------------------------------------------------------
    # Internals

    @staticmethod
    def _resolve_path(path: str, root: str) -> str:
        """Resolve a repo-relative path (starting with /) against the project root."""
        if os.path.isabs(path) and not os.path.exists(path):
            return os.path.join(root, path.lstrip('/'))
        return path