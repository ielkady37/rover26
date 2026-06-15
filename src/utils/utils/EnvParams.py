#!/usr/bin/env python3
"""EnvParams – environment loader for the rover26 stack.

Automatically selects the correct ``.env`` file at import time:

1. ``.env.rasp``  – if found (production Pi deployment)
2. ``.env.local`` – fallback (developer / dev-container)

Searches upward from this file's location until it finds the project
root (identified by the presence of a ``config/`` directory), then looks
for the env files there.

All values are loaded into ``os.environ`` (so subprocesses inherit them)
and are accessible via typed helpers::

    from utils.EnvParams import EnvParams

    ep = EnvParams()
    host = ep.get('ROVER_REDIS_HOST', 'localhost')
    port = ep.get_int('ROVER_REDIS_PORT', 6379)
    debug = ep.get_bool('ROVER_DEBUG', False)

The class is a **singleton** – the env file is parsed only once no matter
how many times ``EnvParams()`` is called.
"""

import os
import pathlib
from typing import Optional


class EnvParams:
    """Auto-selecting, singleton .env loader.

    Priority
    --------
    1. ``.env.rasp``  (present on the Raspberry Pi)
    2. ``.env.local`` (developer machine / dev-container)

    If neither file exists all ``get*`` calls return their supplied
    defaults and a warning is printed once.
    """

    _instance: Optional['EnvParams'] = None
    _loaded: bool = False

    # Path that was actually loaded (None if no file was found)
    env_path: Optional[pathlib.Path] = None

    def __new__(cls) -> 'EnvParams':
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self) -> None:
        if self._loaded:
            return
        EnvParams._loaded = True

        root = self._find_project_root()
        chosen = self._select_env_file(root)

        if chosen is None:
            print(
                '\033[93m[EnvParams] WARNING: neither .env.rasp nor .env.local '
                f'found under {root}. '
                'All env values will fall back to built-in defaults.\033[0m'
            )
        else:
            self._parse(chosen)
            EnvParams.env_path = chosen
            print(f'\033[92m[EnvParams] Loaded env from: {chosen}\033[0m')

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def get(self, key: str, default: str = '') -> str:
        """Return ``os.environ[key]`` or *default* if not set."""
        return os.environ.get(key, default)

    def get_int(self, key: str, default: int = 0) -> int:
        """Return the env value cast to ``int``, or *default*."""
        try:
            return int(os.environ[key])
        except (KeyError, ValueError):
            return default

    def get_float(self, key: str, default: float = 0.0) -> float:
        """Return the env value cast to ``float``, or *default*."""
        try:
            return float(os.environ[key])
        except (KeyError, ValueError):
            return default

    def get_bool(self, key: str, default: bool = False) -> bool:
        """Return the env value as a ``bool``.

        ``'true'``, ``'1'``, ``'yes'`` (case-insensitive) → ``True``.
        Anything else → ``False``.
        """
        val = os.environ.get(key)
        if val is None:
            return default
        return val.strip().lower() in ('true', '1', 'yes')

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _find_project_root() -> pathlib.Path:
        """Walk upward from this file until a ``config/`` directory is found."""
        current = pathlib.Path(__file__).resolve()
        for parent in current.parents:
            if (parent / 'config').is_dir():
                return parent
        # Fallback: four levels up from this file
        return pathlib.Path(__file__).resolve().parents[3]

    @staticmethod
    def _select_env_file(root: pathlib.Path) -> Optional[pathlib.Path]:
        """Return the first existing env file in priority order."""
        for name in ('.env.rasp', '.env.local'):
            candidate = root / name
            if candidate.exists():
                return candidate
        return None

    @staticmethod
    def _parse(path: pathlib.Path) -> None:
        """Parse *path* and inject every key into ``os.environ``.

        Keys already present in the environment (e.g. set by the shell
        before launch) are **not** overwritten, so a real env var always
        wins over the file.
        """
        with open(path) as fh:
            for line in fh:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                key, _, value = line.partition('=')
                key = key.strip()
                value = value.strip()
                if key:
                    os.environ.setdefault(key, value)