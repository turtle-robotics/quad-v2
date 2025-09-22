#!/usr/bin/python3
"""
Simple tests for the color scheme system
"""

import os
import sys
import json
import tempfile

# Add the current directory to Python path so we can import our modules
sys.path.insert(0, os.path.dirname(__file__))

from color_scheme import ColorSchemeManager, ColorScheme, ColorType, hex_to_rgb


def test_hex_to_rgb():
    """Test hex to RGB conversion"""
    assert hex_to_rgb("#FF0000") == (255, 0, 0)
    assert hex_to_rgb("#00FF00") == (0, 255, 0)
    assert hex_to_rgb("#0000FF") == (0, 0, 255)
    assert hex_to_rgb("#FFFFFF") == (255, 255, 255)
    assert hex_to_rgb("#000000") == (0, 0, 0)
    print("✅ hex_to_rgb tests passed")


def test_color_scheme():
    """Test ColorScheme class"""
    scheme = ColorScheme(
        name="test_scheme",
        description="Test scheme",
        colors={
            ColorType.CHASSIS.value: "#FF0000",
            ColorType.LEG_UPPER.value: "#00FF00"
        }
    )
    
    assert scheme.name == "test_scheme"
    assert scheme.description == "Test scheme"
    assert scheme.colors[ColorType.CHASSIS.value] == "#FF0000"
    
    # Test dict conversion
    scheme_dict = scheme.to_dict()
    restored_scheme = ColorScheme.from_dict(scheme_dict)
    assert restored_scheme.name == scheme.name
    assert restored_scheme.colors == scheme.colors
    print("✅ ColorScheme tests passed")


def test_color_scheme_manager():
    """Test ColorSchemeManager with temporary config file"""
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        config_file = f.name
    
    try:
        manager = ColorSchemeManager(config_file)
        
        # Test default schemes are loaded
        schemes = manager.list_schemes()
        assert "turtle_green" in schemes
        assert "classic_black_red" in schemes
        assert len(schemes) >= 5
        
        # Test setting current scheme
        assert manager.set_current_scheme("turtle_green")
        assert manager.current_scheme == "turtle_green"
        
        # Test getting current scheme
        current = manager.get_current_scheme()
        assert current is not None
        assert current.name == "turtle_green"
        
        # Test getting color for component
        chassis_color = manager.get_color_for_component(ColorType.CHASSIS)
        assert chassis_color is not None
        assert chassis_color.startswith("#")
        
        # Test invalid scheme
        assert not manager.set_current_scheme("nonexistent_scheme")
        
        print("✅ ColorSchemeManager tests passed")
        
    finally:
        # Clean up temporary file
        if os.path.exists(config_file):
            os.remove(config_file)


def test_configuration_persistence():
    """Test that configuration is saved and loaded properly"""
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        config_file = f.name
    
    try:
        # Create manager and set a scheme
        manager1 = ColorSchemeManager(config_file)
        manager1.set_current_scheme("industrial_blue")
        
        # Create new manager instance to test loading
        manager2 = ColorSchemeManager(config_file)
        assert manager2.current_scheme == "industrial_blue"
        
        # Test that config file exists and has correct content
        assert os.path.exists(config_file)
        with open(config_file, 'r') as f:
            config = json.load(f)
            assert config["current_scheme"] == "industrial_blue"
        
        print("✅ Configuration persistence tests passed")
        
    finally:
        if os.path.exists(config_file):
            os.remove(config_file)


def run_all_tests():
    """Run all tests"""
    print("Running color scheme system tests...")
    
    test_hex_to_rgb()
    test_color_scheme()
    test_color_scheme_manager()
    test_configuration_persistence()
    
    print("\n🎉 All tests passed!")


if __name__ == "__main__":
    run_all_tests()