#!/usr/bin/env python3
import yaml
import pathlib

class Configurator():
    BUTTONS = "joystick_buttons"
    PINS = "hardware_pins"
    PID_PARAMS = "pid_ks"
    LOCOMOTION = "locomotion"
    MISSION = "mission"
    CAMERAS = "cameras"
    FACE_RECOGNITION = "face_recognition"
    POTHOLES_DETECTION = "potholes_detection"
    LANE_DETECTION = "lane_detection"
    
    def __init__(self):
        self.__configFile = ''
    
    @staticmethod
    def getProjectRoot():
        # Traverse upward until we find a directory with /config folder
        current = pathlib.Path(__file__).resolve()
        for parent in current.parents:
            if (parent / 'config').is_dir():
                return str(parent)
        raise FileNotFoundError("Could not find project root with /config directory")
    def __raiseTypeError(self,data_type):
        consts = [attr for attr in dir(self) if not callable(getattr(self, attr)) and not attr.startswith("_")]
        raise TypeError(f"Config file of type {data_type} doesn't exist, only {', '.join(consts)} are allowed.")
    
    def getConfigsNames(self):
        consts = [attr for attr in dir(self) if not callable(getattr(self, attr)) and not attr.startswith("_")]
        return [getattr(self, attr) for attr in consts]

    def __getYamlFile(self, data_type):
        project_root = Configurator.getProjectRoot()
        config_filename = None
        
        if data_type == Configurator.BUTTONS:
            config_filename = Configurator.BUTTONS
        elif data_type == Configurator.PINS:
            config_filename = Configurator.PINS
        elif data_type == Configurator.PID_PARAMS:
            config_filename = Configurator.PID_PARAMS
        elif data_type == Configurator.LOCOMOTION:
            config_filename = Configurator.LOCOMOTION
        elif data_type == Configurator.MISSION:
            config_filename = Configurator.MISSION
        elif data_type == Configurator.CAMERAS:
            config_filename = Configurator.CAMERAS
        elif data_type == Configurator.FACE_RECOGNITION:
            config_filename = Configurator.FACE_RECOGNITION
        elif data_type == Configurator.POTHOLES_DETECTION:
            config_filename = Configurator.POTHOLES_DETECTION
        elif data_type == Configurator.LANE_DETECTION:
            config_filename = Configurator.LANE_DETECTION
        else:
            self.__raiseTypeError(data_type)
        
        if config_filename:
            self.__configFile = f"{project_root}/config/{config_filename}.yaml"
    def fetchData(self,data_type):
        try:
            self.__getYamlFile(data_type)
            with open(self.__configFile, 'r') as file:
                data = yaml.safe_load(file)
                return data
        except FileNotFoundError:
            print(f"Error: The file '{self.__configFile}' was not found.")
        except yaml.YAMLError:
            print("Error: Failed to parse the YAML file.")
        except TypeError as e:
            print(e)
    def setConfig(self, data_type, new_data):
        """
        Update the YAML configuration file with new_data.
        Only updates the keys provided in new_data and keeps other keys intact.

        :param data_type: Type of configuration (e.g., "cameras", "joystick_buttons")
        :param new_data: Dictionary containing the new key-value pairs to update.
        """
        try:
            self.__getYamlFile(data_type)

            # Load existing data
            try:
                with open(self.__configFile, 'r') as file:
                    existing_data = yaml.safe_load(file) or {}  # Handle empty file case
            except FileNotFoundError:
                existing_data = {}
                print(f"Warning: {self.__configFile} not found. A new file will be created.")

            # Update existing data with new_data (merge dictionaries)
            updated_data = {**existing_data, **new_data}

            # Write back to file
            with open(self.__configFile, 'w') as file:
                yaml.safe_dump(updated_data, file, default_flow_style=False)

            print(f"Configuration for '{data_type}' updated successfully.")

        except yaml.YAMLError as e:
            print(f"Error: Failed to write YAML data. Details: {e}")
        except TypeError as e:
            print(e)#!/usr/bin/env python3
import yaml
import pathlib

class Configurator():
    BUTTONS = "joystick_buttons"
    PINS = "hardware_pins"
    PID_PARAMS = "pid_ks"
    LOCOMOTION = "locomotion"
    MISSION = "mission"
    
    def __init__(self):
        self.__configFile = ''
    
    @staticmethod
    def getProjectRoot():
        # Traverse upward until we find a directory with /config folder
        current = pathlib.Path(__file__).resolve()
        for parent in current.parents:
            if (parent / 'config').is_dir():
                return str(parent)
        raise FileNotFoundError("Could not find project root with /config directory")
    def __raiseTypeError(self,data_type):
        consts = [attr for attr in dir(self) if not callable(getattr(self, attr)) and not attr.startswith("_")]
        raise TypeError(f"Config file of type {data_type} doesn't exist, only {', '.join(consts)} are allowed.")
    
    def getConfigsNames(self):
        consts = [attr for attr in dir(self) if not callable(getattr(self, attr)) and not attr.startswith("_")]
        return [getattr(self, attr) for attr in consts]

    def __getYamlFile(self, data_type):
        project_root = Configurator.getProjectRoot()
        config_filename = None
        
        if data_type == Configurator.BUTTONS:
            config_filename = Configurator.BUTTONS
        elif data_type == Configurator.PINS:
            config_filename = Configurator.PINS
        elif data_type == Configurator.PID_PARAMS:
            config_filename = Configurator.PID_PARAMS
        elif data_type == Configurator.LOCOMOTION:
            config_filename = Configurator.LOCOMOTION
        elif data_type == Configurator.MISSION:
            config_filename = Configurator.MISSION
        else:
            self.__raiseTypeError(data_type)
        
        if config_filename:
            self.__configFile = f"{project_root}/config/{config_filename}.yaml"
    def fetchData(self,data_type):
        try:
            self.__getYamlFile(data_type)
            with open(self.__configFile, 'r') as file:
                data = yaml.safe_load(file)
                return data
        except FileNotFoundError:
            print(f"Error: The file '{self.__configFile}' was not found.")
        except yaml.YAMLError:
            print("Error: Failed to parse the YAML file.")
        except TypeError as e:
            print(e)
    def setConfig(self, data_type, new_data):
        """
        Update the YAML configuration file with new_data.
        Only updates the keys provided in new_data and keeps other keys intact.

        :param data_type: Type of configuration (e.g., "cameras", "joystick_buttons")
        :param new_data: Dictionary containing the new key-value pairs to update.
        """
        try:
            self.__getYamlFile(data_type)

            # Load existing data
            try:
                with open(self.__configFile, 'r') as file:
                    existing_data = yaml.safe_load(file) or {}  # Handle empty file case
            except FileNotFoundError:
                existing_data = {}
                print(f"Warning: {self.__configFile} not found. A new file will be created.")

            # Update existing data with new_data (merge dictionaries)
            updated_data = {**existing_data, **new_data}

            # Write back to file
            with open(self.__configFile, 'w') as file:
                yaml.safe_dump(updated_data, file, default_flow_style=False)

            print(f"Configuration for '{data_type}' updated successfully.")

        except yaml.YAMLError as e:
            print(f"Error: Failed to write YAML data. Details: {e}")
        except TypeError as e:
            print(e)