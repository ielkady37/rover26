#!/usr/bin/env python3
"""
Camera / Layout / Stream / Controller / PID service server.
Now PID params are stored in YAML instead of JSON.
"""

import os
import rclpy
from rclpy.node import Node

import yaml  # ✅ CHANGED (was json)

from interfaces.srv import (
    GetLayout,
    SetLayout,
    GetConfig,
    SetConfig,
    GetStream,
    GetActiveController,
    GetPIDStatus,
)

_DB = os.path.expanduser('~/rover26/src/control/control/database')
_PID_PATH = os.path.expanduser('~/rover26/config/pid_ks.yaml') 


# ─────────────────────────────────────────────
# YAML helpers (NEW)
# ─────────────────────────────────────────────
def _load_yaml(path: str) -> dict:
    if not os.path.exists(path):
        return {}
    with open(path, 'r') as f:
        data = yaml.safe_load(f) or {}
        return data


def _save_yaml(path: str, data: dict) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)
    print(f"[YAML] Saved {path}")


# ─────────────────────────────────────────────
# JSON helpers (unchanged for other configs)
# ─────────────────────────────────────────────
import json

def _load(filename: str) -> dict:
    path = os.path.join(_DB, filename)
    with open(path, 'r') as f:
        return json.load(f)


def _save(filename: str, data: dict) -> None:
    path = os.path.join(_DB, filename)
    with open(path, 'w') as f:
        json.dump(data, f, indent=4)
    print(f"[DB] Saved {filename}")


# ── Load JSON files ─────────────────────────
try:
    CAMERAS_LAYOUT = _load('cameras.json')
except Exception as e:
    raise RuntimeError(e)

try:
    STREAM_DATA = _load('streams.json')
except Exception as e:
    raise RuntimeError(e)

try:
    CONTROLLER_LAYOUT = _load('controller.json')
except Exception as e:
    raise RuntimeError(e)

# ── PID LOAD (NOW YAML) ─────────────────────
try:
    PID_PARAMS = _load_yaml(_PID_PATH)
    print(f"[YAML] Loaded pid_params.yaml")
except Exception as e:
    raise RuntimeError(f"Failed to load PID YAML: {e}")


class CameraControlServer(Node):

    def __init__(self):
        super().__init__('camera_control_server')

        self._layouts = {
            'cameras': CAMERAS_LAYOUT,
            'controller': CONTROLLER_LAYOUT,
        }

        self._stream_data = STREAM_DATA
        self._active_controller = 'ps4'

        # ✅ PID now YAML-backed
        self._pid_params = PID_PARAMS

        # services
        self.create_service(GetLayout, '/getLayoutService', self._get_layout_cb)
        self.create_service(SetLayout, '/setLayoutService', self._set_layout_cb)
        self.create_service(GetConfig, '/getConfigService', self._get_config_cb)
        self.create_service(SetConfig, '/setConfigService', self._set_config_cb)
        self.create_service(GetStream, '/getStreamService', self._get_stream_cb)
        self.create_service(GetActiveController, '/get_active_controller', self._get_active_controller_cb)
        self.create_service(GetPIDStatus, '/getPIDStatus', self._get_pid_status_cb)

        self.get_logger().info("CameraControlServer READY (PID uses YAML)")

    # ── GET CONFIG ───────────────────────────
    def _get_config_cb(self, request, response):
        name = request.config_name

        if name == 'cameras':
            response.config_object = json.dumps(self._stream_data)

        elif name == 'controller':
            response.config_object = json.dumps(self._layouts.get('controller', {}))

        # ✅ PID from YAML
        elif name == 'pid_params':
            response.config_object = json.dumps(self._pid_params)
            self.get_logger().info("GetConfig: pid_params (YAML)")

        else:
            response.config_object = ''
            self.get_logger().warn(f"Unknown config {name}")

        return response

    # ── SET CONFIG ───────────────────────────
    def _set_config_cb(self, request, response):
        try:
            name = request.config_name
            parsed = json.loads(request.config_object)

            if name == 'cameras':
                self._stream_data = parsed
                _save('streams.json', parsed)

            elif name == 'controller':
                self._layouts['controller'] = parsed
                _save('controller.json', parsed)

            # ✅ PID SAVE TO YAML
            elif name == 'pid_params':
                self._pid_params = parsed
                _save_yaml(_PID_PATH, parsed)
                self.get_logger().info("PID saved to YAML")

            else:
                response.success = False
                return response

            response.success = True

        except Exception as e:
            self.get_logger().error(str(e))
            response.success = False

        return response

    # ── OTHER CALLBACKS ───────────────────────
    def _get_layout_cb(self, request, response):
        response.layout_object = json.dumps(
            self._layouts.get(request.layout_name, {})
        )
        return response

    def _set_layout_cb(self, request, response):
        parsed = json.loads(request.layout_object)
        self._layouts[request.layout_name] = parsed
        _save(f"{request.layout_name}.json", parsed)
        response.success = True
        return response

    def _get_stream_cb(self, _req, res):
        res.cameras_data = json.dumps(self._stream_data)
        return res

    def _get_active_controller_cb(self, _req, res):
        res.active_controller = self._active_controller
        return res

    def _get_pid_status_cb(self, _req, res):
        res.status = True
        return res

def main(args=None):
    rclpy.init(args=args)
    node = CameraControlServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()