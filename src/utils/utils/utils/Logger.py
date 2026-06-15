#!/usr/bin/env python3
"""Rover structured logger.

Features
--------
- Colored console output:  succ=green  info=white  warn=yellow  err=red
- Optional per-class file logging under  log/<category>/<ClassName>.log
- Publishes every log entry to the Redis Stream ``rover:logs`` so that
  :class:`control.nodes.LoggerNode` can relay them to the ``/rover/logs``
  ROS topic.  Console and file output always work even when Redis is down.

Redis Stream key:  ``rover:logs``
Stream entry fields:
    level         – SUCC | INFO | WARN | ERR
    source_class  – name of the class that called the logger
    category      – nodes | services | helpers | general | …
    message       – log text
    timestamp     – ISO-8601 string (e.g. 2026-05-03T12:34:56.789)

The stream is trimmed to at most :data:`_STREAM_MAX_LEN` entries so it
does not grow without bound.

Usage::

    from utils.Logger import RoverLogger

    class MyService:
        def __init__(self):
            self._log = RoverLogger()

        def start(self):
            self._log.info("Service starting")
            self._log.succ("Service ready")

        def handle_fault(self):
            self._log.warn("Unexpected value received")
            self._log.err("Critical fault – stopping")
"""

import inspect
import os
import threading
from datetime import datetime
from enum import Enum
from typing import Optional


# ---------------------------------------------------------------------------
# ANSI colour helpers
# ---------------------------------------------------------------------------

_RESET = '\033[0m'
_BOLD = '\033[1m'

# Max entries kept in the Redis stream before trimming
_STREAM_MAX_LEN = int(os.getenv('ROVER_LOG_STREAM_MAX', '50000'))
_STREAM_KEY = os.getenv('ROVER_LOG_STREAM_KEY', 'rover:logs')


class LogLevel(Enum):
    """Log severity levels with associated label and ANSI colour code."""

    SUCCESS = ('SUCC', '\033[92m')   # bright green
    INFO = ('INFO', '\033[97m')      # bright white
    WARN = ('WARN', '\033[93m')      # bright yellow
    ERROR = ('ERR ', '\033[91m')     # bright red

    def __init__(self, label: str, color: str) -> None:
        self.label = label
        self.color = color


# ---------------------------------------------------------------------------
# Caller introspection helpers
# ---------------------------------------------------------------------------

_KNOWN_CATEGORIES = ('nodes', 'services', 'helpers', 'DTOs', 'mocks', 'interface')


def _infer_category(frame) -> str:
    """Derive the log category from the caller's file path."""
    try:
        filepath = inspect.getfile(frame[0]).replace('\\', '/')
        for cat in _KNOWN_CATEGORIES:
            if f'/{cat}/' in filepath:
                return cat
    except (TypeError, OSError):
        pass
    return 'general'


def _infer_class_name(frame) -> str:
    """Return the caller's class name, falling back to the function name."""
    local_self = frame[0].f_locals.get('self')
    if local_self is not None:
        return type(local_self).__name__
    local_cls = frame[0].f_locals.get('cls')
    if local_cls is not None:
        return local_cls.__name__
    return frame[3] or 'unknown'


# ---------------------------------------------------------------------------
# RoverLogger
# ---------------------------------------------------------------------------

class RoverLogger:
    """Per-class structured logger.

    Instantiate once per class (typically in ``__init__``)::

        self._log = RoverLogger()                    # file + Redis (default)
        self._log = RoverLogger(file_logging=False)  # console + Redis only

    Methods
    -------
    succ(msg)   – green   SUCCESS
    info(msg)   – white   INFO
    warn(msg)   – yellow  WARNING
    err(msg)    – red     ERROR
    """

    # Shared file-handle registry across all instances (process-wide)
    _handles: dict = {}
    _file_lock = threading.Lock()

    def __init__(self, file_logging: bool = True) -> None:
        self._file_logging = file_logging

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def succ(self, message: str) -> None:
        """Log a SUCCESS entry (green)."""
        self._log(LogLevel.SUCCESS, message)

    def info(self, message: str) -> None:
        """Log an INFO entry (white)."""
        self._log(LogLevel.INFO, message)

    def warn(self, message: str) -> None:
        """Log a WARNING entry (yellow)."""
        self._log(LogLevel.WARN, message)

    def err(self, message: str) -> None:
        """Log an ERROR entry (red)."""
        self._log(LogLevel.ERROR, message)

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _log(self, level: LogLevel, message: str) -> None:
        # stack: 0=_log, 1=succ/info/warn/err, 2=actual caller
        stack = inspect.stack()
        frame = stack[2] if len(stack) > 2 else stack[-1]

        class_name = _infer_class_name(frame)
        category = _infer_category(frame)
        now = datetime.now()
        ts = now.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

        # --- Console ---
        print(
            f"{level.color}{_BOLD}[{level.label}]{_RESET}"
            f"{level.color} {ts} | {class_name:>28} | {message}{_RESET}"
        )

        # --- File ---
        if self._file_logging:
            self._write_file(category, class_name, level, ts, message)

        # --- Redis Stream ---
        self._push_redis(level, class_name, category, message, now)

    def _push_redis(
        self,
        level: LogLevel,
        class_name: str,
        category: str,
        message: str,
        now: datetime,
    ) -> None:
        """Push this entry to the Redis stream ``rover:logs``."""
        try:
            from utils.RedisClient import RedisClient
            rc = RedisClient()
            if not rc.available:
                return
            rc.client.xadd(
                _STREAM_KEY,
                {
                    'level': level.label.strip(),
                    'source_class': class_name,
                    'category': category,
                    'message': message,
                    'timestamp': now.isoformat(timespec='milliseconds'),
                },
                maxlen=_STREAM_MAX_LEN,
                approximate=True,
            )
        except Exception:
            # Never let Redis errors interrupt the caller
            pass

    def _write_file(
        self,
        category: str,
        class_name: str,
        level: LogLevel,
        ts: str,
        message: str,
    ) -> None:
        try:
            log_dir = self._resolve_log_dir(category)
            log_path = os.path.join(log_dir, f'{class_name}.log')
            with RoverLogger._file_lock:
                if log_path not in RoverLogger._handles:
                    os.makedirs(log_dir, exist_ok=True)
                    # line-buffered so crashes don't swallow tail lines
                    RoverLogger._handles[log_path] = open(  # noqa: WPS515
                        log_path, 'a', buffering=1
                    )
                fh = RoverLogger._handles[log_path]
                fh.write(f'[{level.label}] {ts} | {class_name} | {message}\n')
        except OSError as exc:
            print(f'\033[91m[RoverLogger] File write error: {exc}\033[0m')

    @staticmethod
    def _resolve_log_dir(category: str) -> str:
        try:
            from utils.Configurator import Configurator
            root = Configurator.getProjectRoot()
        except Exception:
            root = os.getcwd()
        return os.path.join(root, 'log', category)
