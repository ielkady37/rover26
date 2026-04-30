#!/usr/bin/env python3
from enum import Enum

class LogSeverity(Enum):
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    FATAL = "FATAL"

    def get_color_code(self):
        """Returns ANSI color code for terminal output"""
        color_map = {
            LogSeverity.INFO: "\033[92m",      # Green
            LogSeverity.WARNING: "\033[93m",   # Yellow
            LogSeverity.ERROR: "\033[91m",     # Red
            LogSeverity.FATAL: "\033[1;91m",   # Bold Red
        }
        return color_map.get(self, "\033[0m")  # Default to reset
    
    def get_hex_color(self):
        """Returns hex color code for GUI output"""
        color_map = {
            LogSeverity.INFO: "#00AA00",       # Green
            LogSeverity.WARNING: "#FFAA00",    # Orange/Yellow
            LogSeverity.ERROR: "#FF0000",      # Red
            LogSeverity.FATAL: "#AA0000",      # Dark Red
        }
        return color_map.get(self, "#000000")  # Default to black