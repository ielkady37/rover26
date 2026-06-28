import cv2
import time
import torch
import numpy as np
import torchreid

from ultralytics import YOLO
from pathlib import Path

# =========================================================
# SETTINGS
# =========================================================

FRAME_WIDTH = 640
FRAME_HEIGHT = 840

SIMILARITY_THRESHOLD = 0.65
LOCK_TIME = 0.5

TARGET_IMAGE = "target 6.jpg"

CALIBRATION_FILE = "fisheye_calibration.npz"
BALANCE = 0.0

# =========================================================
# LOAD YOLO
# =========================================================

model = YOLO("yolov8n.pt")

# =========================================================
# LOAD REID MODEL
# =========================================================

device = "cuda" if torch.cuda.is_available() else "cpu"

extractor = torchreid.utils.FeatureExtractor(
    model_name='osnet_ain_x1_0',
    model_path='',
    device=device
)

print("ReID model loaded on", device)

# =========================================================
# LOAD TARGET IMAGE
# =========================================================

target_img = cv2.imread(TARGET_IMAGE)

if target_img is None:
    print(f"Cannot find {TARGET_IMAGE}")
    exit()

# Detect person inside target image
results = model(target_img, verbose=False)

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
    print("No person found in target image")
    exit()

# Extract target features
target_vector = (
    extractor([target_crop])[0]
    .cpu()
    .numpy()
)

target_vector = (
    target_vector
    / np.linalg.norm(target_vector)
)

target_vector = target_vector.reshape(1, -1)

print("Target image loaded successfully.")



# =========================================================
# FISHEYE CALIBRATION
# =========================================================

def scale_intrinsics(K, from_size, to_size):

    from_w, from_h = from_size
    to_w, to_h = to_size

    sx = to_w / from_w
    sy = to_h / from_h

    K_scaled = K.copy()

    K_scaled[0, 0] *= sx
    K_scaled[0, 2] *= sx
    K_scaled[1, 1] *= sy
    K_scaled[1, 2] *= sy

    return K_scaled


def build_maps(calib_path, balance):

    data = np.load(calib_path)

    K_raw = data["K"].astype(np.float64)
    D = data["D"].astype(np.float64)

    calib_size = tuple(int(x) for x in data["image_size"])

    K = scale_intrinsics(
        K_raw,
        calib_size,
        (FRAME_WIDTH, FRAME_HEIGHT)
    )

    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K,
        D,
        (FRAME_WIDTH, FRAME_HEIGHT),
        np.eye(3),
        balance=balance
    )

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K,
        D,
        np.eye(3),
        new_K,
        (FRAME_WIDTH, FRAME_HEIGHT),
        cv2.CV_32FC1
    )

    return map1, map2


map1, map2 = build_maps(
    Path(CALIBRATION_FILE),
    BALANCE
)


# =========================================================
# CAMERA
# =========================================================

cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

if not cap.isOpened():

    print("Camera 1 failed. Trying camera 0...")

    cap = cv2.VideoCapture(0)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if not cap.isOpened():
        print("Cannot open camera.")
        exit()

actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

print(f"Camera initialized: {actual_width} x {actual_height}")


# =========================================================
# VARIABLES
# =========================================================
confirm_counter = 0
prev_time = time.time()
smoothed_embedding = None



# =========================================================
# MAIN LOOP
# =========================================================
while True:

    ret, frame = cap.read()

    if not ret:
        print("Failed to grab frame.")
        break

    frame = cv2.remap(
        frame,
        map1,
        map2,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT
    )

    processing_frame = frame.copy()

    results = model(
        processing_frame,
        stream=True,
        verbose=False
    )

    best_score = 0
    best_box = None
    second_score = 0

    scores = []

    # ==========================================
    # Compute similarity
    # ==========================================
    for r in results:
        for box in r.boxes:

            if int(box.cls[0]) != 0:
                continue

            x1, y1, x2, y2 = map(
                int,
                box.xyxy[0]
            )

            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(frame.shape[1], x2)
            y2 = min(frame.shape[0], y2)

            person_crop = processing_frame[
                y1:y2,
                x1:x2
            ]

            if (
                person_crop.size == 0
                or person_crop.shape[0] < 50
                or person_crop.shape[1] < 20
            ):
                continue

            try:

                person_vector = (
                    extractor([person_crop])[0]
                    .cpu()
                    .numpy()
                )

                person_vector = (
                    person_vector
                    / np.linalg.norm(person_vector)
                )

                person_vector = (
                    person_vector.reshape(1, -1)
                )

                similarity = np.dot(
                    target_vector,
                    person_vector.T
                )[0][0]

                scores.append(
                    (
                        similarity,
                        (x1, y1, x2, y2)
                    )
                )

            except:
                continue

    # ==========================================
    # Choose best target
    # ==========================================
    if len(scores) > 0:

        scores.sort(
            key=lambda x: x[0],
            reverse=True
        )

        best_score, best_box = scores[0]

        if len(scores) > 1:
            second_score = scores[1][0]

        print(
            f"BEST={best_score:.3f} "
            f"SECOND={second_score:.3f}"
        )

        if (
            best_score > SIMILARITY_THRESHOLD
            and
            (
                len(scores) == 1
                or
                (best_score - second_score) > 0.03
            )
        ):

            bx1, by1, bx2, by2 = best_box

            cv2.rectangle(
                frame,
                (bx1, by1),
                (bx2, by2),
                (0, 255, 0),
                4
            )

            cv2.putText(
                frame,
                f"TARGET {best_score:.2f}",
                (bx1, by1 - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            confirm_counter += 1

        else:
            confirm_counter = 0

    else:
        confirm_counter = 0

    # ==========================================
    # HARD LOCK
    # ==========================================
    if confirm_counter >= 5:

        cv2.putText(
            frame,
            "!!! HARD LOCK !!!",
            (200, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.2,
            (0, 255, 0),
            3
        )

    cv2.imshow(
        "UGVC Advanced Tracking",
        frame
    )

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()