#!/usr/bin/env python3
"""Thread-safe Redis singleton for the rover stack.

All rover components (logger, health monitor, services) share a single
connection pool via this singleton.

Usage::

    from utils.RedisClient import RedisClient

    r = RedisClient().client
    r.xadd('rover:logs', {'level': 'INFO', 'message': 'hello'})

Configuration (environment variables, all optional):
    ROVER_REDIS_HOST   – default: localhost
    ROVER_REDIS_PORT   – default: 6379
    ROVER_REDIS_DB     – default: 0
    ROVER_REDIS_PASS   – default: (none)
"""

import os
import threading
from typing import Optional

try:
    import redis
    _REDIS_AVAILABLE = True
except ImportError:
    _REDIS_AVAILABLE = False


class RedisClient:
    """Thread-safe singleton wrapper around a Redis connection pool.

    All rover processes share one pool.  Call :attr:`client` to get
    the underlying ``redis.Redis`` instance.

    If the ``redis`` package is not installed, or the server is unreachable,
    :attr:`available` is ``False`` and :attr:`client` is ``None``.
    """

    _instance: Optional['RedisClient'] = None
    _class_lock = threading.Lock()

    def __new__(cls) -> 'RedisClient':
        if cls._instance is None:
            with cls._class_lock:
                if cls._instance is None:
                    inst = super().__new__(cls)
                    inst._client = None
                    inst._available = False
                    inst._connect()
                    cls._instance = inst
        return cls._instance

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def available(self) -> bool:
        """True when the Redis connection is healthy."""
        return self._available

    @property
    def client(self) -> Optional['redis.Redis']:
        """The ``redis.Redis`` client, or ``None`` if unavailable."""
        return self._client

    def ping(self) -> bool:
        """Send a PING and return True if the server responds."""
        try:
            if self._client:
                self._client.ping()
                return True
        except Exception:
            self._available = False
        return False

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _connect(self) -> None:
        if not _REDIS_AVAILABLE:
            print(
                '\033[93m[RedisClient] redis-py not installed – '
                'ROS topic publishing and Redis health checks disabled. '
                'Run: pip install redis\033[0m'
            )
            return

        host = os.getenv('ROVER_REDIS_HOST', 'localhost')
        port = int(os.getenv('ROVER_REDIS_PORT', '6379'))
        db = int(os.getenv('ROVER_REDIS_DB', '0'))
        password = os.getenv('ROVER_REDIS_PASS') or None

        try:
            pool = redis.ConnectionPool(
                host=host,
                port=port,
                db=db,
                password=password,
                decode_responses=True,
                socket_connect_timeout=2,
                socket_timeout=2,
                retry_on_timeout=True,
                max_connections=20,
            )
            client = redis.Redis(connection_pool=pool)
            client.ping()   # fail fast if server is down
            self._client = client
            self._available = True
        except Exception as exc:
            print(
                f'\033[93m[RedisClient] Could not connect to Redis '
                f'({host}:{port}) – {exc}. '
                f'ROS topic publishing disabled.\033[0m'
            )
