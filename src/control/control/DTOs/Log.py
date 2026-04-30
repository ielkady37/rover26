#!/usr/bin/env python3
from DTOs.LogSeverity import LogSeverity
from datetime import datetime

RESET_COLOR = "\033[0m"

class Log:
    def __init__(self, severity: LogSeverity, message: str, component_name: str):
        self.severity = severity
        self.message = message
        self.component_name = component_name
        self.timestamp = datetime.now()

    def toDictionary(self):
        """Converts the Log object to a dictionary for JSON serialization."""
        return {
            "severity": self.severity.value,
            "message": self.message,
            "component_name": self.component_name,
            "timestamp": self.timestamp.isoformat(),
            "color": self.severity.get_hex_color()
        }

    def __str__(self):
        """Returns a colored string representation of the log."""
        color_code = self.severity.get_color_code()
        return f"{color_code}[{self.timestamp}] [{self.severity.value}] [{self.component_name}] {self.message}{RESET_COLOR}"
    
    def to_string_plain(self):
        """Returns a plain string representation without color codes."""
        return f"[{self.timestamp}] [{self.severity.value}] [{self.component_name}] {self.message}"
    
    def get_color(self):
        """Returns the hex color code for this log's severity level."""
        return self.severity.get_hex_color()