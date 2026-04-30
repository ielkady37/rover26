#!/usr/bin/env python3
import os

class EnvParams:
    def __init__(self, env_filename=".env"):
        """
        Initialize the EnvParams class by loading key-value pairs
        from the specified .env file into class attributes.

        :param env_filename: Name of the .env file to load.
        """
        self.env_filename = env_filename
        self.env_path = self._find_env_file()
        if self.env_path:
            self._load_env_file()
        else:
            raise FileNotFoundError(f"{self.env_filename} file not found in the directory hierarchy.")

    def _find_env_file(self):
        """
        Traverse up the directory hierarchy to locate the .env file.

        :return: Absolute path to the .env file or None if not found.
        """
        current_dir = os.path.abspath(os.path.dirname(__file__))

        while current_dir:  # Traverse up to the root directory
            env_path = os.path.join(current_dir, self.env_filename)
            if os.path.exists(env_path):
                return env_path
            # Move up one directory
            parent_dir = os.path.dirname(current_dir)
            if parent_dir == current_dir:  # Reached root directory
                break
            current_dir = parent_dir

        return None

    def _load_env_file(self):
        """
        Load key-value pairs from the .env file into os.environ and class attributes.
        """
        with open(self.env_path) as env_file:
            for line in env_file:
                # Strip whitespace and skip comments/empty lines
                line = line.strip()
                if line and not line.startswith("#"):
                    key, _, value = line.partition("=")
                    key, value = key.strip(), value.strip()
                    # Add to os.environ
                    os.environ[key] = value
                    # Add as a class attribute
                    setattr(self, key, value)