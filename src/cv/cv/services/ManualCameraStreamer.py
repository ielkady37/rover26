import cv2
import os
import signal
import pyshine as ps
import time
import logging
from multiprocessing import Process, Event
from utils.EnvParams import EnvParams

logger = logging.getLogger(__name__)


class _ReadRetryCapture:
    def __init__(self, capture, max_retries=5, retry_delay_sec=0.02) -> None:
        self._capture = capture
        self._max_retries = max(1, int(max_retries))
        self._retry_delay_sec = max(0.0, float(retry_delay_sec))
        self._last_good_frame = None  # Cache of last known-good frame

    def read(self):
        for attempt in range(self._max_retries):
            ok, frame = self._capture.read()
            if ok and frame is not None and frame.size > 0:
                self._last_good_frame = frame  # Update cache
                return True, frame
            if attempt < self._max_retries - 1 and self._retry_delay_sec > 0:
                time.sleep(self._retry_delay_sec)

        # All retries failed — return last good frame to prevent empty-frame encoding
        if self._last_good_frame is not None:
            logger.warning(
                "All read retries failed — reusing last good frame to prevent stream drop."
            )
            return True, self._last_good_frame

        return False, None

    def __getattr__(self, name):
        return getattr(self._capture, name)


class ManualCameraStreamer:
    # How long to wait between full reconnect attempts
    RECONNECT_DELAY_SEC = 3.0
    # How many times to retry opening the device before giving up
    MAX_OPEN_RETRIES = 10

    def __init__(self, cameraIndex, port, format="MJPG") -> None:
        self.address = EnvParams().get("WEB_DOMAIN", "localhost")
        self.cameraIndex = cameraIndex
        self.width = 1280
        self.height = 720
        self.FPS = 60
        self.port = port
        self.process = None
        self.format_type = (format or "MJPG").upper()
        self.warmup_frames = 20
        self.read_retries = 5
        self.read_retry_delay_sec = 0.02
        self._stop_event = Event()  # Signals the child to exit cleanly

    def setFrameSize(self, width=1280, height=720):
        self.width = width
        self.height = height

    def setFPS(self, FPS):
        self.FPS = FPS

    def __setCVAttrs(self, capture):
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)      # Minimal buffer — reduces stale frame buildup
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.FPS)

    def _candidate_devices(self):
        """
        Build an ordered list of /dev/videoN paths to try opening.

        UVC cameras commonly expose a real capture node alongside a
        metadata-only sibling node (e.g. /dev/video2 + /dev/video3) that
        share the same USB interface number, so a udev rule can end up
        pointing a friendly symlink (like /dev/Fisheye) at either one
        depending on enumeration order. cv2.VideoCapture opens the
        metadata node "successfully" in some drivers but never yields
        frames, or fails isOpened() outright in others — either way we
        shouldn't retry the same broken node 10 times and give up when a
        working sibling sits right next to it.

        If self.cameraIndex is an int or doesn't resolve to /dev/videoN,
        this just returns [self.cameraIndex] unchanged (original behaviour).
        """
        target = self.cameraIndex
        if not isinstance(target, str):
            return [target]

        resolved = os.path.realpath(target)
        base_dir = os.path.dirname(resolved)
        name = os.path.basename(resolved)
        if not name.startswith("video") or not name[5:].isdigit():
            return [target]

        node_num = int(name[5:])
        # Try the resolved node first (whatever the symlink currently
        # points at), then its immediate siblings — the metadata node is
        # conventionally the next index up, but check both directions.
        candidate_nums = [node_num, node_num + 1, node_num - 1]

        seen = set()
        candidates = []
        for n in candidate_nums:
            if n < 0 or n in seen:
                continue
            seen.add(n)
            path = os.path.join(base_dir, f"video{n}")
            if os.path.exists(path):
                candidates.append(path)
        return candidates or [target]

    def _probe_capture(self, capture):
        """Quick check that a capture handle is a real, frame-producing node."""
        if not capture.isOpened():
            return False
        ok, frame = capture.read()
        return bool(ok and frame is not None and frame.size > 0)

    def _fourcc_to_str(self, capture):
        fourcc_code = int(capture.get(cv2.CAP_PROP_FOURCC))
        return "".join([chr((fourcc_code >> 8 * i) & 0xFF) for i in range(4)])

    def _warmup_capture(self, capture):
        successful_reads = 0
        for _ in range(self.warmup_frames):
            ok, frame = capture.read()
            if ok and frame is not None and frame.size > 0:
                successful_reads += 1
            else:
                time.sleep(self.read_retry_delay_sec)
        if successful_reads == 0:
            raise RuntimeError("Camera warmup failed: no valid frames received.")

    def __setup_camera(self, device):
        if self.format_type == "H264":
            return self._setup_h264(device)
        elif self.format_type == "MJPG":
            return self._setup_mjpg(device)
        elif self.format_type in ("YUYV", "YUY2"):
            return self._setup_yuyv(device)
        else:
            raise ValueError("Unsupported format. Choose one of: 'MJPG', 'YUYV', 'H264'.")

    def _setup_mjpg(self, device):
        print(f"Initializing camera '{device}' with MJPEG format...")
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "video_codec;mjpeg"
        capture = cv2.VideoCapture(device)
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        capture.set(cv2.CAP_PROP_FOURCC, fourcc)
        return capture

    def _setup_yuyv(self, device):
        print(f"Initializing camera '{device}' from YUYV profile...")
        capture = cv2.VideoCapture(device)
        mjpg_fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        capture.set(cv2.CAP_PROP_FOURCC, mjpg_fourcc)
        if self._fourcc_to_str(capture) == 'MJPG':
            self.format_type = 'MJPG'
            print("Camera accepted MJPG.")
            return capture
        yuyv_fourcc = cv2.VideoWriter_fourcc(*'YUYV')
        capture.set(cv2.CAP_PROP_FOURCC, yuyv_fourcc)
        self.format_type = 'YUYV'
        print("Falling back to YUYV.")
        return capture

    def _setup_h264(self, device):
        print(f"Initializing camera '{device}' with H.264 format...")
        pipelines = [
            (
                f"v4l2src device={device} ! "
                f"video/x-h264, width={self.width}, height={self.height}, framerate={self.FPS}/1 ! "
                "h264parse ! avdec_h264 ! videoconvert ! appsink drop=1 sync=false"
            ),
            (
                f"v4l2src device={device} ! "
                f"image/jpeg, width={self.width}, height={self.height}, framerate={self.FPS}/1 ! "
                "jpegdec ! videoconvert ! appsink drop=1 sync=false"
            ),
            (
                f"v4l2src device={device} ! "
                f"video/x-raw, width={self.width}, height={self.height}, framerate={self.FPS}/1 ! "
                "videoconvert ! appsink drop=1 sync=false"
            ),
        ]
        for pipeline in pipelines:
            capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if capture.isOpened():
                ok, _ = capture.read()
                if ok:
                    return capture
            capture.release()

        return cv2.VideoCapture(device)

    def _try_open_capture(self):
        """
        Attempt to open the camera device with retries.
        Returns a valid, opened capture or raises RuntimeError.
        This is the single place that handles 'device busy after restart'.

        Each attempt walks every candidate device node (the configured
        node plus its immediate /dev/videoN siblings) so a udev symlink
        that happens to point at a metadata-only node — rather than the
        actual capture node — doesn't just retry the wrong node 10 times
        and give up.
        """
        candidates = self._candidate_devices()
        if len(candidates) > 1:
            print(f"[{self.cameraIndex}] Will try candidate nodes: {candidates}")

        for attempt in range(1, self.MAX_OPEN_RETRIES + 1):
            if self._stop_event.is_set():
                raise RuntimeError("Stop requested during device open.")

            for device in candidates:
                try:
                    capture = self.__setup_camera(device)
                    if self._probe_capture(capture):
                        if device != candidates[0]:
                            print(
                                f"[{self.cameraIndex}] Configured node was wrong "
                                f"(metadata/no-frames) — using '{device}' instead."
                            )
                        return capture
                    capture.release()
                except Exception as e:
                    print(
                        f"[{self.cameraIndex}] Open attempt {attempt}/{self.MAX_OPEN_RETRIES} "
                        f"on '{device}' failed: {e}"
                    )

            print(
                f"[{self.cameraIndex}] No candidate device ready. "
                f"Retrying in {self.RECONNECT_DELAY_SEC}s... "
                f"({attempt}/{self.MAX_OPEN_RETRIES})"
            )
            time.sleep(self.RECONNECT_DELAY_SEC)

        raise RuntimeError(
            f"Camera '{self.cameraIndex}' could not be opened after {self.MAX_OPEN_RETRIES} attempts "
            f"(tried: {candidates})."
        )

    def __run(self):
        # The child forks after the node installs its ROS shutdown handlers;
        # reset them so the stop event (and a real SIGTERM) control this process.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        while not self._stop_event.is_set():
            raw_capture = None
            server = None
            try:
                StreamProps = ps.StreamProps
                StreamProps.set_Page(StreamProps, "")
                address = (self.address, self.port)

                raw_capture = self._try_open_capture()

                StreamProps.set_Mode(StreamProps, 'cv2')
                self.__setCVAttrs(raw_capture)
                self._warmup_capture(raw_capture)

                retry_capture = _ReadRetryCapture(
                    raw_capture,
                    max_retries=self.read_retries,
                    retry_delay_sec=self.read_retry_delay_sec,
                )
                StreamProps.set_Capture(StreamProps, retry_capture)
                StreamProps.set_Quality(StreamProps, 90)

                width = raw_capture.get(cv2.CAP_PROP_FRAME_WIDTH)
                height = raw_capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
                fps = raw_capture.get(cv2.CAP_PROP_FPS)
                format_used = self._fourcc_to_str(raw_capture)
                print(
                    f"[{self.cameraIndex}] Streaming — "
                    f"{int(width)}x{int(height)} @ {int(fps)}fps [{format_used}]"
                )

                server = ps.Streamer(address, StreamProps)
                # serve_forever() blocks — run it in a thread so we can stop it cleanly
                import threading
                server_thread = threading.Thread(target=server.serve_forever, daemon=True)
                server_thread.start()

                # Poll stop event instead of blocking forever
                while not self._stop_event.is_set():
                    self._stop_event.wait(timeout=0.5)

                server.shutdown()       # signals serve_forever() to exit
                server_thread.join(timeout=3)

            except Exception as e:
                print(f"[{self.cameraIndex}] Stream error: {e}")

            finally:
                # Now this ALWAYS runs because we're no longer stuck in serve_forever()
                if raw_capture is not None:
                    try:
                        raw_capture.release()
                        print(f"[{self.cameraIndex}] Device released.")
                    except Exception as e:
                        print(f"[{self.cameraIndex}] Warning: release failed: {e}")

                if server is not None:
                    try:
                        server.socket.close()
                    except Exception:
                        pass

            if not self._stop_event.is_set():
                print(f"[{self.cameraIndex}] Restarting stream in {self.RECONNECT_DELAY_SEC}s...")
                self._stop_event.wait(timeout=self.RECONNECT_DELAY_SEC)

    def stream(self):
        self._stop_event.clear()
        self.process = Process(target=self.__run, daemon=True)
        self.process.start()

    def closeStream(self):
        # 1. Signal the child loop to stop
        self._stop_event.set()

        if self.process is None:
            return

        # 2. Let the child exit its loop and release the device in its finally block
        self.process.join(timeout=5)

        # 3. Escalate only if it didn't exit gracefully
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=2)
        if self.process.is_alive():
            print(f"[{self.cameraIndex}] Force killing stream process.")
            self.process.kill()
            self.process.join(timeout=2)
        self.process = None