# QuadV2

## Setup Workspace
- Create a python virtual environment, install all dependencies in requirements.txt
- Copy the fdcanusb udev rule (for linux users) where it belongs (read the comments in the file linked below)
    - https://github.com/mjbots/fdcanusb/blob/master/70-fdcanusb.rules
- https://github.com/mjbots/moteus/blob/main/docs/getting_started.md

## Color Scheme System
The robot includes a color scheme management system for 3D printing consistency:

### Quick Start
```bash
# List available color schemes
python3 color_scheme.py list

# Preview a color scheme
python3 color_scheme.py preview turtle_green

# Set active color scheme
python3 color_scheme.py set turtle_green

# Generate 3D printing guide
python3 robot_colors.py guide

# Export colors for other tools
python3 robot_colors.py export
```

### Available Schemes
- `classic_black_red`: Classic black chassis with red accents
- `turtle_green`: TURTLE Robotics themed green and black
- `industrial_blue`: Industrial blue and silver theme  
- `sunset_orange`: Warm sunset orange and black
- `arctic_white`: Clean white and blue arctic theme

### Component Types
- **Chassis**: Main robot body structure
- **Upper Legs**: Shoulder and thigh components
- **Lower Legs**: Shin and foot components
- **Joints**: Motor mounts and pivot points
- **Accents**: Decorative elements and logos


## Notes
- position range is -1 to 1 revolutions by default
- to load stored config run `conf load` in tview gui
- if things are funky try `conf load` a few times   
- need to figure out how to `conf load` from a python script
- moteus console: python3 -m moteus_gui.tview
- calibrate: python3 -m moteus.moteus_tool --target 1 --calibrate

## To Reset Configs
- to load default config in tview gui write `conf default` then `conf write`
- to get correct configs copy   
```
conf set servo.pid_position.kp 100
conf set servo.pid_position.ki 1
conf set servo.pid_position.kd 10
conf set servopos.position_min nan
conf set servopos.position_max nan
conf set motor_position.rotor_to_output_ratio 0.0526315789
conf set servo.max_velocity 10.0
conf set servo.default_velocity_limit 8.0
conf set servo.default_accel_limit 20.0
```
into tview gui then run `conf write` 


## Known Working Steps
1. `conf defaut` in tview
2. calibrate `python3 -m moteus.moteus_tool --target 1 --calibrate` in terminal
3. zero offset `python3 -m moteus.moteus_tool --target 1 --zero-offset`
4. apply config in tview 
```
conf set servo.pid_position.kp 100
conf set servo.pid_position.ki 1
conf set servo.pid_position.kd 10
conf set servopos.position_min nan
conf set servopos.position_max nan
conf set motor_position.rotor_to_output_ratio 0.0526315789
conf set servo.max_velocity 10.0
conf set servo.default_velocity_limit 8.0
conf set servo.default_accel_limit 20.0
```
5. `conf write` in tview
6. `conf load` in tview
7. `d pos nan 1 nan` in tview

## Known Working Steps Python Script
Assuming the motor controller has the configuration stored on it from the above section (run again if not)

If fault code is 36, the motor needs to be calibrated with the moteus tool. Run `python3 -m moteus.moteus_tool --target 1 --calibrate` in your terminal, **THIS WILL SPIN THE MOTOR A LOT**

If something stops working, unplug and replug

1. run test.py
