#!/usr/bin/env python3
import sys
from unittest.mock import MagicMock

# Stub out hardware-only modules so they can be imported on non-hardware hosts
for _mod in ("spidev", "smbus2"):
    if _mod not in sys.modules:
        sys.modules[_mod] = MagicMock()
