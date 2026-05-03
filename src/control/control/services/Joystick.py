#!/usr/bin/env python3

from multiprocessing.shared_memory import SharedMemory
import pickle
import threading
from utils.services.Configurator import Configurator
import atexit
import signal
import time
from utils.services.Logger import Logger
from utils.DTOs.LogSeverity import LogSeverity

class CJoystick:
    _instance = None
    def __new__(cls, *args, **kwargs):
        if cls._instance is None:  # Ensure a single instance
            cls._instance = super(CJoystick, cls).__new__(cls)
        return cls._instance  # Always return the same instance

    def __init__(self, is_writer=False,shared_memory_name="joystick_data", buffer_size=1024):
        if not hasattr(self, "_initialized"):  # Ensure __init__ runs only once
            self._initialized = True
            self.shared_memory_name = shared_memory_name
            self.buffer_size = buffer_size
            self.is_writer = is_writer  # Distinguish between writer and reader
            self.lock = threading.Lock()
            self.previous_button_states = {}
            self.button_press_timestamps = {}  # Store button press timestamps
            self.last_click_time = {}  # Store the last registered click time
            self.hold_counts = {}  # Track how long a button is held
            self.start_time = {}  # Store when the first click happened
            self.waiting_period = {}  # Flag to track waiting period
            self.click_counts = {}  # Track clicks before returning a count

            try:
                # Try to attach to an existing shared memory block
                self.shared_memory = SharedMemory(name=self.shared_memory_name)
                Logger.logToFile(LogSeverity.INFO, "Shared memory attached successfully.", "CJoystick")
            except FileNotFoundError:
                # If not found, create a new shared memory block
                self.shared_memory = SharedMemory(name=self.shared_memory_name, create=True, size=self.buffer_size)
                self.is_writer = True  # This instance will act as the writer
                Logger.logToFile(LogSeverity.INFO, "Shared memory created successfully.", "CJoystick")

            atexit.register(self.cleanup)
            signal.signal(signal.SIGINT, self._signal_cleanup)
            signal.signal(signal.SIGTERM, self._signal_cleanup)
            self.__constructConstants()

    @classmethod
    def __constructConstants(cls):
        """
        Dynamically create class constants from the configuration file.
        This ensures that button mappings are automatically assigned as class attributes.

        Example Config:
        {
            "button_0": "FLASH",
            "button_x": "LEFTGRIPPER_OPEN",
            "button_l1": "LEFTGRIPPER_CLOSE",
            "button_4": "RIGHTGRIPPER_OPEN"
        }
        """
        joystickButtons = Configurator().fetchData(Configurator.BUTTONS)
        if joystickButtons:
            for button_key, button_name in joystickButtons.items():
                if button_key.startswith("button"):  # Ensure valid button key format
                    try:
                        button_number = button_key.replace("button", "")
                        # print("ana ahooo: ")
                        # print(button_number)
                        setattr(cls, button_name, button_number)
                        setattr(cls, f"_{button_number}", button_number)  # Add support for _X button notation
                    except ValueError:
                        Logger.logToFile(LogSeverity.ERROR, f"Invalid button number format: '{button_key}'", "CJoystick")
                        continue  # Skip invalid keys


    def _serialize_data(self, data):
        """
        Serialize data using pickle.
        """
        return pickle.dumps(data)

    def _deserialize_data(self, buffer):
        """
        Deserialize data using pickle.
        """
        return pickle.loads(buffer.rstrip(b"\x00"))  # Remove padding

    def updateData(self, buttons_data, axis_data):
        """
        Write the joystick data (buttons and axes) to shared memory (writer).
        
        Parameters:
            buttons_data (object): Button data (can be any serializable object like a dictionary).
            axis_data (list): List containing joystick axis values [left_x, left_y, right_x, right_y].
        """
        if not self.is_writer:
            Logger.logToFile(LogSeverity.ERROR, "Only the writer instance can update data.", "CJoystick")
            raise PermissionError("Only the writer instance can update data.")

        try:
            with self.lock:
                data_with_timestamp = {
                    "timestamp": time.time(),
                    "buttons": buttons_data,
                    "axes": axis_data
                }
                serialized_data = self._serialize_data(data_with_timestamp)

                if len(serialized_data) > self.buffer_size:
                    Logger.logToFile(LogSeverity.ERROR, "Data exceeds shared memory buffer size.", "CJoystick")
                    raise ValueError("Data exceeds shared memory buffer size.")

                self.shared_memory.buf[:len(serialized_data)] = serialized_data
                self.shared_memory.buf[len(serialized_data):] = b"\x00" * (self.buffer_size - len(serialized_data))
        except Exception as e:
            Logger.logToFile(LogSeverity.ERROR, f"Shared memory update failed: {e}", "CJoystick")
            self.is_writer = False  # Fallback to prevent invalid writes


    def __getData(self):
        """
        Read the joystick data from shared memory (reader).
        
        Returns:
            object: The deserialized joystick data, or None if invalid or unavailable.
        """
        retries = 5
        while retries > 0:
            try:
                if not hasattr(self, "shared_memory") or self.shared_memory is None:
                    self.shared_memory = SharedMemory(name=self.shared_memory_name)

                with self.lock:
                    data_with_timestamp = self._deserialize_data(bytes(self.shared_memory.buf))
                    return data_with_timestamp
            except FileNotFoundError:
                Logger.logToFile(LogSeverity.ERROR, "Shared memory not found. Retrying...", "CJoystick")
                print("Shared memory not found. Retrying...")
                time.sleep(1)
                retries -= 1
            except (pickle.UnpicklingError, EOFError, KeyError):
                Logger.logToFile(LogSeverity.ERROR, "Failed to deserialize data.", "CJoystick")
                return None

        print("Failed to connect to shared memory after retries.")
        return None

    def isPressed(self, button_name, time_window=0.4):
        """
        Count the number of times a button was pressed within the last `time_window` seconds.
        The count remains as long as the button is being held and also detects consecutive presses.
        If the last press is still held, it does not reset to zero. The function keeps returning 0 until
        the time window expires, then returns the count. If the button is released, the count resets to zero.

        Parameters:
            button_name (str): The name of the button constant (e.g., LEFTGRIPPER_OPEN, _1).
            time_window (float): The time window in seconds to count presses.

        Returns:
            int: The number of presses detected within the specified time window.

        Raises:
            ValueError: If the button name is not defined.
        """
        data = self.__getData()
        if data is None or "buttons" not in data:
            return 0  # No data available

        # Resolve button number (either "_X" or BUTTON_NAME)
        if button_name.startswith("_"):
            try:
                button_number = int(button_name[1:])  # Extract the number
            except ValueError:
                Logger.logToFile(LogSeverity.FATAL, f"Invalid button number format: '{button_name}'", "CJoystick")
                raise ValueError(f"Invalid button number format: '{button_name}'")
        else:
            button_number = getattr(self, button_name, None)

        if button_number is None:
            Logger.logToFile(LogSeverity.FATAL, f"Button name '{button_name}' is not defined.", "CJoystick")
            raise ValueError(f"Button name '{button_name}' is not defined.")

        button_key = f"button{button_number}"

        # Ensure the button exists in received data
        if button_key not in data["buttons"]:
            Logger.logToFile(LogSeverity.WARNING, f"Button key '{button_key}' not found in data.", "CJoystick")
            return 0

        current_state = data["buttons"].get(button_key, False)
        current_time = time.time()

        # Initialize tracking structures if not present
        if button_key not in self.previous_button_states:
            self.previous_button_states[button_key] = False
            self.button_press_timestamps[button_key] = []
            self.last_click_time[button_key] = 0
            self.hold_counts[button_key] = 0
            self.start_time[button_key] = 0
            self.waiting_period[button_key] = True

        previous_state = self.previous_button_states[button_key]

        # Detect state change: False → True (rising edge)
        if current_state and not previous_state:
            self.button_press_timestamps[button_key].append(current_time)
            self.last_click_time[button_key] = current_time  # Store the last click time
            self.hold_counts[button_key] += 1  # Increase count when pressed
            if self.start_time[button_key] == 0:
                self.start_time[button_key] = current_time  # Start the waiting period
                self.waiting_period[button_key] = True

        # Update previous state
        self.previous_button_states[button_key] = current_state

        # If the waiting period is still active, return 0
        if self.waiting_period[button_key] and (current_time - self.start_time[button_key] < time_window):
            return 0

        # End the waiting period once the time window expires
        self.waiting_period[button_key] = False

        # If the button is released, reset to 0
        if not current_state:
            self.hold_counts[button_key] = 0
            self.start_time[button_key] = 0
            self.waiting_period[button_key] = True
            return 0

        return self.hold_counts[button_key]

    
    def isClicked(self, button_name, time_window=0.4):
        """
        Count the number of times a button was clicked (state changed from False to True)
        within the last `time_window` seconds. The function keeps returning 0 until
        the time window expires, then returns the count, regardless of whether the button is held.

        Parameters:
            button_name (str): The name of the button constant (e.g., LEFTGRIPPER_OPEN, _1).
            time_window (float): The time window in seconds to count clicks.

        Returns:
            int: The number of state changes (False → True) detected within the specified time window.

        Raises:
            ValueError: If the button name is not defined.
        """
        data = self.__getData()
        if data is None or "buttons" not in data:
            return 0  # No data available

        # Resolve button number (either "_X" or BUTTON_NAME)
        if button_name.startswith("_"):
            try:
                button_number = int(button_name[1:])  # Extract the number
            except ValueError:
                Logger.logToFile(LogSeverity.FATAL, f"Invalid button number format: '{button_name}'", "CJoystick")
                raise ValueError(f"Invalid button number format: '{button_name}'")
        else:
            button_number = getattr(self, button_name, None)

        if button_number is None:
            Logger.logToFile(LogSeverity.FATAL, f"Button name '{button_name}' is not defined.", "CJoystick")
            raise ValueError(f"Button name '{button_name}' is not defined.")

        button_key = f"button{button_number}"

        # Ensure the button exists in received data
        if button_key not in data["buttons"]:
            Logger.logToFile(LogSeverity.WARNING, f"Button key '{button_key}' not found in data.", "CJoystick")
            return 0

        current_state = data["buttons"].get(button_key, False)
        current_time = time.time()

        # Initialize tracking structures if not present
        if button_key not in self.previous_button_states:
            self.previous_button_states[button_key] = False
            self.button_press_timestamps[button_key] = []
            self.last_click_time[button_key] = 0
            self.click_counts[button_key] = 0
            self.start_time[button_key] = 0
            self.waiting_period[button_key] = True

        previous_state = self.previous_button_states[button_key]

        # Detect state change: False → True (rising edge)
        if current_state and not previous_state:
            self.button_press_timestamps[button_key].append(current_time)
            self.last_click_time[button_key] = current_time  # Store the last click time
            self.click_counts[button_key] += 1  # Increase count when clicked
            if self.start_time[button_key] == 0:
                self.start_time[button_key] = current_time  # Start the waiting period
                self.waiting_period[button_key] = True

        # Update previous state
        self.previous_button_states[button_key] = current_state

        # If the waiting period is still active, keep storing the count but return 0
        if self.waiting_period[button_key] and (current_time - self.start_time[button_key] < time_window):
            return 0

        # End the waiting period once the time window expires
        self.waiting_period[button_key] = False

        # Store the final count to return exactly once
        final_count = self.click_counts[button_key]
        
        # Reset all tracking variables for the next detection cycle
        self.click_counts[button_key] = 0
        self.start_time[button_key] = 0
        self.waiting_period[button_key] = True

        return final_count
    
    
    def getAxis(self):
        """
        Returns an array of joystick axis values.
        
        Returns:
            dictionary: Dictionary of joystick axis values {left_x_axis: 0.0, left_y_axis: 0.0, right_x_axis: 0.0, right_y_axis: 0.0}.
        """
        data = self.__getData()
        if data is None or "axes" not in data:
            return {"left_x_axis": 0.0, "left_y_axis": 0.0, "right_x_axis": 0.0, "right_y_axis": 0.0}  # Default values if no data available
        return data["axes"]

    def _signal_cleanup(self, signum, frame):
        """
        Perform cleanup when a termination signal is received.
        """
        print(f"Received termination signal {signum}. Cleaning up shared memory.")
        self.cleanup()

    def cleanup(self):
        """
        Clean up shared memory (only the writer should unlink it).
        """
        if self.is_writer:
            self.shared_memory.unlink()
        self.shared_memory.close()
