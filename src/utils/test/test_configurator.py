#!/usr/bin/env python3
import unittest
import yaml
import tempfile
import shutil
import pathlib
from pathlib import Path
import sys

# Add parent directory to path to import Configurator
sys.path.insert(0, str(Path(__file__).parent.parent))
from utils.Configurator import Configurator


class TestConfigurator(unittest.TestCase):
    """Test cases for the Configurator class"""

    @classmethod
    def setUpClass(cls):
        """Set up test fixtures for all tests"""
        # Create a temporary directory for test config files
        cls.test_dir = tempfile.mkdtemp()
        cls.config_dir = Path(cls.test_dir) / 'config'
        cls.config_dir.mkdir()

        # Create test config files
        cls.test_configs = {
            'hardware_pins.yaml': {
                'motor_right_pwm': 1,
                'motor_right_dir': 2,
                'motor_left_pwm': 3,
                'motor_left_dir': 4,
            },
            'joystick_buttons.yaml': {
                'button_l1': 'ACTION1',
                'button_r1': 'ACTION2',
                'button_x': 'ACTION3',
            },
            'pid_ks.yaml': {
                'yaw_KP': 1.5,
                'yaw_KI': 0.5,
                'yaw_KD': 0.2,
            }
        }

        for filename, content in cls.test_configs.items():
            filepath = cls.config_dir / filename
            with open(filepath, 'w') as f:
                yaml.safe_dump(content, f)

    @classmethod
    def tearDownClass(cls):
        """Clean up temporary test directory"""
        shutil.rmtree(cls.test_dir)

    def test_get_project_root_exists(self):
        """Test that getProjectRoot() returns a valid path"""
        root = Configurator.getProjectRoot()
        self.assertIsNotNone(root)
        self.assertTrue(Path(root).exists())
        self.assertTrue((Path(root) / 'config').is_dir())
        print(f"Project root found at: {root}")

    def test_get_configs_names(self):
        """Test that getConfigsNames() returns correct config names"""
        config = Configurator()
        names = config.getConfigsNames()
        expected_names = ['joystick_buttons', 'hardware_pins', 'pid_ks']
        # Check all expected names are present (order might vary)
        for name in expected_names:
            self.assertIn(name, names)
        print(f"Config names: {names}")

    def test_fetch_data_buttons(self):
        """Test fetching joystick_buttons configuration"""
        config = Configurator()
        data = config.fetchData(Configurator.BUTTONS)
        self.assertIsNotNone(data)
        self.assertIsInstance(data, dict)
        print(f"Joystick buttons config: {data}")

    def test_fetch_data_pins(self):
        """Test fetching hardware_pins configuration"""
        config = Configurator()
        data = config.fetchData(Configurator.PINS)
        self.assertIsNotNone(data)
        self.assertIsInstance(data, dict)
        print(f"Hardware pins config: {data}")

    def test_fetch_data_pid_params(self):
        """Test fetching pid_ks configuration"""
        config = Configurator()
        data = config.fetchData(Configurator.PID_PARAMS)
        self.assertIsNotNone(data)
        self.assertIsInstance(data, dict)
        print(f"PID parameters config: {data}")

    def test_fetch_data_invalid_type(self):
        """Test fetching with invalid config type"""
        config = Configurator()
        # Should not raise, but should print error
        result = config.fetchData("invalid_config_type")
        self.assertIsNone(result)
        print("Invalid config type handled correctly.")

    def test_set_config_buttons(self):
        """Test updating joystick_buttons configuration"""
        config = Configurator()
        new_data = {'button_l1': 'NEW_ACTION', 'button_r1': 'ANOTHER_ACTION'}
        config.setConfig(Configurator.BUTTONS, new_data)
        
        # Verify the data was updated
        updated_data = config.fetchData(Configurator.BUTTONS)
        self.assertEqual(updated_data['button_l1'], 'NEW_ACTION')
        self.assertEqual(updated_data['button_r1'], 'ANOTHER_ACTION')

    def test_set_config_pins(self):
        """Test updating hardware_pins configuration"""
        config = Configurator()
        new_data = {'motor_right_pwm': 10, 'motor_left_pwm': 11}
        config.setConfig(Configurator.PINS, new_data)
        
        # Verify the data was updated
        updated_data = config.fetchData(Configurator.PINS)
        self.assertEqual(updated_data['motor_right_pwm'], 10)
        self.assertEqual(updated_data['motor_left_pwm'], 11)

    def test_set_config_pid_params(self):
        """Test updating pid_ks configuration"""
        config = Configurator()
        new_data = {'yaw_KP': 2.5, 'yaw_KI': 1.0}
        config.setConfig(Configurator.PID_PARAMS, new_data)
        
        # Verify the data was updated
        updated_data = config.fetchData(Configurator.PID_PARAMS)
        self.assertEqual(updated_data['yaw_KP'], 2.5)
        self.assertEqual(updated_data['yaw_KI'], 1.0)

    def test_set_config_preserves_existing_keys(self):
        """Test that setConfig preserves keys not being updated"""
        config = Configurator()
        
        # Get initial data
        initial_data = config.fetchData(Configurator.BUTTONS)
        initial_keys = set(initial_data.keys())
        
        # Update only one key
        config.setConfig(Configurator.BUTTONS, {'button_x': 'UPDATED'})
        
        # Verify all original keys still exist
        updated_data = config.fetchData(Configurator.BUTTONS)
        updated_keys = set(updated_data.keys())
        self.assertEqual(initial_keys, updated_keys)

    def test_set_config_add_new_key(self):
        """Test that setConfig can add new keys"""
        config = Configurator()
        new_key = 'new_button_test'
        new_value = 'NEW_TEST_ACTION'
        
        config.setConfig(Configurator.BUTTONS, {new_key: new_value})
        
        # Verify new key was added
        updated_data = config.fetchData(Configurator.BUTTONS)
        self.assertIn(new_key, updated_data)
        self.assertEqual(updated_data[new_key], new_value)

    def test_raise_type_error_invalid_type(self):
        """Test that invalid config type raises TypeError"""
        config = Configurator()
        # Calling private method to test error handling
        with self.assertRaises(TypeError):
            config._Configurator__raiseTypeError("invalid_type")

    def test_multiple_instances_independent(self):
        """Test that multiple Configurator instances are independent"""
        config1 = Configurator()
        config2 = Configurator()
        
        data1 = config1.fetchData(Configurator.PINS)
        data2 = config2.fetchData(Configurator.PINS)
        
        self.assertEqual(data1, data2)
        self.assertIsNot(data1, data2)

    def test_yaml_file_structure(self):
        """Test that YAML files maintain proper structure"""
        config = Configurator()
        data = config.fetchData(Configurator.PINS)
        
        # Verify expected keys exist
        self.assertIn('motor_right_pwm', data)
        self.assertIn('motor_left_pwm', data)

    def test_fetch_returns_none_on_error(self):
        """Test that fetchData returns None on file not found"""
        config = Configurator()
        result = config.fetchData("nonexistent_config")
        self.assertIsNone(result)

    def test_get_configs_names_count(self):
        """Test that getConfigsNames returns correct number of configs"""
        config = Configurator()
        names = config.getConfigsNames()
        self.assertEqual(len(names), 3)


if __name__ == '__main__':
    test = TestConfigurator()
    test.test_get_project_root_exists()
    test.test_get_configs_names()
    test.test_fetch_data_buttons()
    test.test_fetch_data_pins()
    test.test_fetch_data_pid_params()
    test.test_fetch_data_invalid_type()
