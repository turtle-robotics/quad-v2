# QuadV2

## Setup Workspace
- Create a python virtual environment, install all dependencies in requirements.txt
- Copy the fdcanusb udev rule (for linux users) where it belongs (read the comments in the file linked below)
    - https://github.com/mjbots/fdcanusb/blob/master/70-fdcanusb.rules
- https://github.com/mjbots/moteus/blob/main/docs/getting_started.md


## Notes
- position range is -1 to 1 revolutions by default
- to load stored config run `conf load` in tview gui
- if things are funky try `conf load` a few times   
- need to figure out how to `conf load` from a python script
- moteus console: python3 -m moteus-tview.gui

## To Reset Configs
- to load default config in tview gui write `conf default` then `conf write`
- to get correct configs copy   
```
conf set servo.pid_position.kp 4.0
conf set servo.pid_position.ki 0.0
conf set servo.pid_position.kd 0.2
conf set servopos.position_min -1.0
conf set servopos.position_max 1.0
conf set motor_position.rotor_to_output_ratio 1.0
conf set servo.max_velocity 10.0
conf set servo.default_velocity_limit 8.0
conf set servo.default_accel_limit 20.0
```
into tview gui then run `conf write` 