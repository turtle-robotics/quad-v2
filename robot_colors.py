#!/usr/bin/python3
"""
Robot Color Integration Example

This script demonstrates how to use the color scheme system
in the context of the QuadV2 robot project.
"""

from color_scheme import ColorSchemeManager, ColorType, hex_to_rgb
import json


def generate_3d_printing_guide():
    """Generate a guide for 3D printing with the selected colors"""
    manager = ColorSchemeManager()
    current_scheme = manager.get_current_scheme()
    
    if not current_scheme:
        print("No color scheme selected. Please run:")
        print("python3 color_scheme.py set <scheme_name>")
        return
    
    print(f"🎨 3D Printing Color Guide - {current_scheme.name}")
    print(f"Description: {current_scheme.description}")
    print("="*60)
    
    components = [
        ("Chassis Frame", ColorType.CHASSIS, "Main robot body structure"),
        ("Upper Leg Segments", ColorType.LEG_UPPER, "Shoulder and thigh components"),
        ("Lower Leg Segments", ColorType.LEG_LOWER, "Shin and foot components"),
        ("Joint Connectors", ColorType.JOINT, "Motor mounts and pivot points"),
        ("Accent Parts", ColorType.ACCENT, "Decorative elements and logos")
    ]
    
    print("\nComponents to Print:")
    for name, color_type, description in components:
        color = current_scheme.colors.get(color_type.value, "#000000")
        rgb = hex_to_rgb(color)
        print(f"\n📦 {name}")
        print(f"   Color: {color} (RGB: {rgb[0]}, {rgb[1]}, {rgb[2]})")
        print(f"   Description: {description}")
    
    print("\n💡 Tips:")
    print("- Use PLA+ or PETG filament for durability")
    print("- Ensure consistent color throughout each component type")
    print("- Print test pieces first to verify colors match your vision")
    print("- Consider the visibility of different components in your design")


def export_color_config():
    """Export color configuration for other tools"""
    manager = ColorSchemeManager()
    current_scheme = manager.get_current_scheme()
    
    if not current_scheme:
        print("No color scheme selected.")
        return
    
    # Export to different formats that might be useful
    config = {
        "robot_colors": {
            "scheme_name": current_scheme.name,
            "scheme_description": current_scheme.description,
            "components": {}
        }
    }
    
    for component_type in ColorType:
        color = current_scheme.colors.get(component_type.value, "#000000")
        rgb = hex_to_rgb(color)
        config["robot_colors"]["components"][component_type.value] = {
            "hex": color,
            "rgb": list(rgb),
            "name": component_type.value.replace('_', ' ').title()
        }
    
    # Save to JSON file
    with open("robot_color_config.json", "w") as f:
        json.dump(config, f, indent=2)
    
    print(f"✅ Color configuration exported to robot_color_config.json")
    print(f"Current scheme: {current_scheme.name}")


def preview_all_schemes():
    """Show previews of all available color schemes"""
    manager = ColorSchemeManager()
    schemes = manager.list_schemes()
    
    print("🌈 All Available Color Schemes Preview")
    print("="*50)
    
    for scheme_name in schemes:
        scheme = manager.get_scheme(scheme_name)
        current_marker = " ⭐ (CURRENT)" if scheme_name == manager.current_scheme else ""
        
        print(f"\n🎨 {scheme.name.upper()}{current_marker}")
        print(f"   {scheme.description}")
        
        # Show a compact color bar
        colors = []
        for component_type in ColorType:
            color = scheme.colors.get(component_type.value, "#000000")
            colors.append(color)
        
        color_bar = " ".join([f"█" for _ in colors])
        print(f"   Colors: {color_bar} {' '.join(colors)}")


def main():
    """Main function for robot color integration"""
    import sys
    
    if len(sys.argv) < 2:
        print("QuadV2 Robot Color Integration")
        print("\nUsage:")
        print("  python3 robot_colors.py guide     - Generate 3D printing color guide")
        print("  python3 robot_colors.py export    - Export color config for other tools")
        print("  python3 robot_colors.py preview   - Preview all available schemes")
        print("\nFirst set a color scheme with:")
        print("  python3 color_scheme.py set <scheme_name>")
        return
    
    command = sys.argv[1]
    
    if command == "guide":
        generate_3d_printing_guide()
    elif command == "export":
        export_color_config()
    elif command == "preview":
        preview_all_schemes()
    else:
        print(f"Unknown command: {command}")


if __name__ == "__main__":
    main()