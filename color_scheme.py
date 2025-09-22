#!/usr/bin/python3
"""
Color Scheme Selection System for QuadV2 Robot

This module provides functionality to choose and manage color schemes
for 3D printed robot parts, with support for visualization and configuration.
"""

import json
import os
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, asdict
from enum import Enum


class ColorType(Enum):
    """Types of components that can be colored"""
    CHASSIS = "chassis"
    LEG_UPPER = "leg_upper" 
    LEG_LOWER = "leg_lower"
    JOINT = "joint"
    ACCENT = "accent"


@dataclass
class ColorScheme:
    """Represents a complete color scheme for the robot"""
    name: str
    description: str
    colors: Dict[str, str]  # ColorType -> hex color
    
    def to_dict(self) -> Dict:
        """Convert to dictionary for JSON serialization"""
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: Dict) -> 'ColorScheme':
        """Create ColorScheme from dictionary"""
        return cls(**data)


class ColorSchemeManager:
    """Manages color schemes for the robot"""
    
    def __init__(self, config_file: str = "robot_colors.json"):
        self.config_file = config_file
        self.schemes: Dict[str, ColorScheme] = {}
        self.current_scheme: Optional[str] = None
        self._load_default_schemes()
        self._load_config()
    
    def _load_default_schemes(self):
        """Load built-in color schemes"""
        default_schemes = [
            ColorScheme(
                name="classic_black_red",
                description="Classic black chassis with red accents",
                colors={
                    ColorType.CHASSIS.value: "#2C2C2C",    # Dark gray/black
                    ColorType.LEG_UPPER.value: "#2C2C2C",  # Dark gray/black
                    ColorType.LEG_LOWER.value: "#2C2C2C",  # Dark gray/black
                    ColorType.JOINT.value: "#808080",      # Medium gray
                    ColorType.ACCENT.value: "#DC143C"      # Crimson red
                }
            ),
            ColorScheme(
                name="turtle_green",
                description="TURTLE Robotics themed green and black",
                colors={
                    ColorType.CHASSIS.value: "#1E3A2E",    # Dark forest green
                    ColorType.LEG_UPPER.value: "#2E8B57",  # Sea green
                    ColorType.LEG_LOWER.value: "#228B22",  # Forest green
                    ColorType.JOINT.value: "#696969",      # Dim gray
                    ColorType.ACCENT.value: "#90EE90"      # Light green
                }
            ),
            ColorScheme(
                name="industrial_blue",
                description="Industrial blue and silver theme",
                colors={
                    ColorType.CHASSIS.value: "#1E3A8A",    # Navy blue
                    ColorType.LEG_UPPER.value: "#3B82F6",  # Blue
                    ColorType.LEG_LOWER.value: "#1D4ED8",  # Blue-700
                    ColorType.JOINT.value: "#C0C0C0",      # Silver
                    ColorType.ACCENT.value: "#60A5FA"      # Blue-400
                }
            ),
            ColorScheme(
                name="sunset_orange",
                description="Warm sunset orange and black",
                colors={
                    ColorType.CHASSIS.value: "#1C1C1C",    # Almost black
                    ColorType.LEG_UPPER.value: "#FF8C00",  # Dark orange
                    ColorType.LEG_LOWER.value: "#FF6347",  # Tomato
                    ColorType.JOINT.value: "#2F2F2F",      # Dark gray
                    ColorType.ACCENT.value: "#FFD700"      # Gold
                }
            ),
            ColorScheme(
                name="arctic_white",
                description="Clean white and blue arctic theme",
                colors={
                    ColorType.CHASSIS.value: "#F8F8FF",    # Ghost white
                    ColorType.LEG_UPPER.value: "#E6E6FA",  # Lavender
                    ColorType.LEG_LOWER.value: "#D3D3D3",  # Light gray
                    ColorType.JOINT.value: "#4682B4",      # Steel blue
                    ColorType.ACCENT.value: "#00BFFF"      # Deep sky blue
                }
            )
        ]
        
        for scheme in default_schemes:
            self.schemes[scheme.name] = scheme
    
    def _load_config(self):
        """Load configuration from file"""
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    data = json.load(f)
                    self.current_scheme = data.get('current_scheme')
                    
                    # Load custom schemes
                    for scheme_data in data.get('custom_schemes', []):
                        scheme = ColorScheme.from_dict(scheme_data)
                        self.schemes[scheme.name] = scheme
                        
            except (json.JSONDecodeError, KeyError) as e:
                print(f"Warning: Could not load config file: {e}")
    
    def save_config(self):
        """Save current configuration to file"""
        custom_schemes = []
        for name, scheme in self.schemes.items():
            # Only save custom schemes (not built-in ones)
            if name not in ['classic_black_red', 'turtle_green', 'industrial_blue', 
                           'sunset_orange', 'arctic_white']:
                custom_schemes.append(scheme.to_dict())
        
        config = {
            'current_scheme': self.current_scheme,
            'custom_schemes': custom_schemes
        }
        
        with open(self.config_file, 'w') as f:
            json.dump(config, f, indent=2)
    
    def list_schemes(self) -> List[str]:
        """Get list of available color scheme names"""
        return list(self.schemes.keys())
    
    def get_scheme(self, name: str) -> Optional[ColorScheme]:
        """Get a specific color scheme"""
        return self.schemes.get(name)
    
    def set_current_scheme(self, name: str) -> bool:
        """Set the current active color scheme"""
        if name in self.schemes:
            self.current_scheme = name
            self.save_config()
            return True
        return False
    
    def get_current_scheme(self) -> Optional[ColorScheme]:
        """Get the currently active color scheme"""
        if self.current_scheme:
            return self.schemes.get(self.current_scheme)
        return None
    
    def add_custom_scheme(self, scheme: ColorScheme):
        """Add a custom color scheme"""
        self.schemes[scheme.name] = scheme
        self.save_config()
    
    def remove_scheme(self, name: str) -> bool:
        """Remove a custom color scheme (cannot remove built-in schemes)"""
        if name in ['classic_black_red', 'turtle_green', 'industrial_blue', 
                   'sunset_orange', 'arctic_white']:
            return False  # Cannot remove built-in schemes
        
        if name in self.schemes:
            del self.schemes[name]
            if self.current_scheme == name:
                self.current_scheme = None
            self.save_config()
            return True
        return False
    
    def get_color_for_component(self, component: ColorType) -> Optional[str]:
        """Get color hex code for a specific component using current scheme"""
        current = self.get_current_scheme()
        if current:
            return current.colors.get(component.value)
        return None


def hex_to_rgb(hex_color: str) -> Tuple[int, int, int]:
    """Convert hex color to RGB tuple"""
    hex_color = hex_color.lstrip('#')
    return tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))


def print_color_preview(hex_color: str, name: str = "") -> str:
    """Generate a simple text preview of a color"""
    rgb = hex_to_rgb(hex_color)
    # Simple brightness calculation for choosing text color
    brightness = (rgb[0] * 299 + rgb[1] * 587 + rgb[2] * 114) / 1000
    text_color = "black" if brightness > 128 else "white"
    
    preview = f"█████ {hex_color}"
    if name:
        preview += f" ({name})"
    return preview


def main():
    """Command line interface for color scheme management"""
    import sys
    
    manager = ColorSchemeManager()
    
    if len(sys.argv) < 2:
        print("QuadV2 Color Scheme Manager")
        print("\nUsage:")
        print("  python3 color_scheme.py list              - List available schemes")
        print("  python3 color_scheme.py show <name>       - Show scheme details")
        print("  python3 color_scheme.py set <name>        - Set active scheme")
        print("  python3 color_scheme.py current           - Show current scheme")
        print("  python3 color_scheme.py preview <name>    - Preview a scheme")
        return
    
    command = sys.argv[1]
    
    if command == "list":
        schemes = manager.list_schemes()
        current = manager.current_scheme
        print("Available color schemes:")
        for name in schemes:
            marker = " (current)" if name == current else ""
            scheme = manager.get_scheme(name)
            print(f"  {name}: {scheme.description}{marker}")
    
    elif command == "show" and len(sys.argv) > 2:
        name = sys.argv[2]
        scheme = manager.get_scheme(name)
        if scheme:
            print(f"\nColor Scheme: {scheme.name}")
            print(f"Description: {scheme.description}")
            print("\nColors:")
            for component, color in scheme.colors.items():
                print(f"  {component.replace('_', ' ').title()}: {print_color_preview(color)}")
        else:
            print(f"Scheme '{name}' not found")
    
    elif command == "set" and len(sys.argv) > 2:
        name = sys.argv[2]
        if manager.set_current_scheme(name):
            print(f"Set active color scheme to: {name}")
        else:
            print(f"Scheme '{name}' not found")
    
    elif command == "current":
        current = manager.get_current_scheme()
        if current:
            print(f"Current scheme: {current.name}")
            print(f"Description: {current.description}")
        else:
            print("No current scheme set")
    
    elif command == "preview" and len(sys.argv) > 2:
        name = sys.argv[2]
        scheme = manager.get_scheme(name)
        if scheme:
            print(f"\n🎨 Color Preview: {scheme.name}")
            print(f"   {scheme.description}")
            print("\n   Robot Component Colors:")
            
            components = [
                ("Chassis", ColorType.CHASSIS),
                ("Upper Legs", ColorType.LEG_UPPER), 
                ("Lower Legs", ColorType.LEG_LOWER),
                ("Joints", ColorType.JOINT),
                ("Accents", ColorType.ACCENT)
            ]
            
            for display_name, component in components:
                color = scheme.colors.get(component.value, "#000000")
                print(f"   {display_name:12}: {print_color_preview(color)}")
            print()
        else:
            print(f"Scheme '{name}' not found")
    
    else:
        print("Unknown command. Use 'python3 color_scheme.py' for usage help.")


if __name__ == "__main__":
    main()