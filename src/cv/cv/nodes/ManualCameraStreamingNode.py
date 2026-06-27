import signal
import time
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from utils.Configurator import Configurator
from cv.services.ManualCameraStreamer import ManualCameraStreamer

SINGLE_REQUIRED_KEYS = ("index", "port", "format", "fps", "width", "height")

DEVICE_SETTLE_SEC = 1.5

# Minimum seconds between captures to prevent duplicate saves from one button press
CAPTURE_COOLDOWN_SEC = 1.0

class ManualCameraStreamingNode(LifecycleNode):
    def __init__(self):
        super().__init__("manual_camera_streaming_node")
        self.configurator = None
        self.camerasDetails = {}
        self.cameraStreamers: list[ManualCameraStreamer] = []
        self._shutdown_requested = False
        self._last_capture_time = 0.0
        self._logger = self.get_logger()

    def request_shutdown(self, reason="Shutdown requested"):
        """Idempotent shutdown hook used by signals and finalizer."""
        if self._shutdown_requested:
            return
        self._logger.warning(reason)
        self.stopAllStreams()

    def __getCameraSteamDetails(self):
        return self.configurator.fetchData(Configurator.CAMERAS)

    def on_configure(self, state):
        """Lifecycle: Configure - load configuration and prepare resources."""
        try:
            self._logger.info("Configuring camera streamer node...")
            self.configurator = Configurator()
            self.camerasDetails = self.__getCameraSteamDetails()
            self._logger.info("Camera configuration loaded successfully.")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self._logger.error(f"Failed to configure: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        """Lifecycle: Activate - start all camera streams."""
        try:
            self._logger.info("Activating camera streamer node...")
            self._shutdown_requested = False
            self.runStreams()
            self._logger.info("All camera streams activated.")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self._logger.error(f"Failed to activate: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state):
        """Lifecycle: Deactivate - stop all camera streams."""
        try:
            self._logger.info("Deactivating camera streamer node...")
            self.stopAllStreams()
            self._logger.info("All camera streams deactivated.")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self._logger.error(f"Failed to deactivate: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_cleanup(self, state):
        """Lifecycle: Cleanup - release resources."""
        try:
            self._logger.info("Cleaning up camera streamer node...")
            self.stopAllStreams()
            self.camerasDetails.clear()
            self.configurator = None
            self._logger.info("Cleanup completed.")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self._logger.error(f"Failed to cleanup: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_shutdown(self, state):
        """Lifecycle: Shutdown - final cleanup."""
        try:
            self._logger.info("Shutting down camera streamer node...")
            self.stopAllStreams()
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self._logger.error(f"Failed to shutdown: {e}")
            return TransitionCallbackReturn.FAILURE


    def __validateKeys(self, camera, details, required_keys):
        missing = [k for k in required_keys if k not in details]
        if missing:
            raise KeyError(
                f"Camera '{camera}' is missing required config keys: {missing}"
            )

    def _start_single(self, camera, details):
        self.__validateKeys(camera, details, SINGLE_REQUIRED_KEYS)
        streamer = ManualCameraStreamer(
            details["index"], details["port"], details["format"]
        )
        streamer.setFPS(details["fps"])
        streamer.setFrameSize(details["width"], details["height"])
        streamer.stream()
        self.cameraStreamers.append(streamer)
        self._logger.info(
            f"Stream started: {camera} | {details['index']} | "
            f"port {details['port']} | "
            f"{details['width']}x{details['height']} @ {details['fps']}fps"
        )

    def runStreams(self):
        for camera, details in self.camerasDetails.items():
            try:
                self._start_single(camera, details)
                time.sleep(3.0)
            except Exception as e:
                self._logger.error(f"Failed to start stream for '{camera}': {e}")

    def stopAllStreams(self):
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        self._logger.warning("Stopping all streams...")

        for streamer in self.cameraStreamers:
            try:
                streamer.closeStream()
            except Exception as e:
                self._logger.error(f"Error closing stream: {e}")

        self.cameraStreamers.clear()

        time.sleep(DEVICE_SETTLE_SEC)
        self._logger.warning("All streams stopped. Devices released.")

    def restartStreams(self):
        """Full stop → settle → restart cycle."""
        self._logger.warning("Restarting all streams...")
        self.stopAllStreams()
        self._shutdown_requested = False
        self.runStreams()


def main(args=None):
    rclpy.init(args=args)
    node = ManualCameraStreamingNode()

    def _on_shutdown(sig, frame):
        node.request_shutdown(
            f"Signal {signal.Signals(sig).name} received - shutting down."
        )
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, _on_shutdown)
    signal.signal(signal.SIGTERM, _on_shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.request_shutdown("KeyboardInterrupt received - stopping streams.")
    finally:
        node.request_shutdown("Finalizing node shutdown.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()