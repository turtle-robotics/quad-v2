# Reference

This is a collection of info that may or may not be important for this project.

## moteus fault codes

From [the docs](https://mjbots.github.io/moteus/protocol/registers/#0x00f-fault-code):


 > A fault code which will be set if the primary mode is 1 (Fault).
 > 
 >  * 32 - calibration fault - the encoder was not able to sense a magnet during calibration
 >  * 33 - motor driver fault - the most common reason for this is undervoltage, moteus attempted to draw more current than the supply could provide. Other electrical faults may also report this error, the drv8323 diagnostic tree has more information.
 >  * 34 - over voltage - the bus voltage exceeded servo.max_voltage. This can happen due to misconfiguration, or if the controller regenerated power with a supply that cannot sink power and no flux braking was configured.
 >  * 35 - encoder fault - the encoder readings are not consistent with a magnet being present.
 >  * 36 - motor not configured - the moteus_tool --calibrate procedure has not been run on this motor.
 >  * 37 - pwm cycle overrun - an internal firmware error
 >  * 38 - over temperature - the maximum configured temperature has been exceeded
 >  * 39 - outside limit - an attempt was made to start position control while outside the bounds configured by servopos.position_min and servopos.position_max.
 >  * 40 - under voltage - the voltage was too low
 >  * 41 - config changed - a configuration value was changed during operation that requires a stop
 >  * 42 - theta invalid - no valid commutation encoder is available
 >  * 43 - position invalid - no valid output encoder is available
 >  * 44 - driver enable fault - the MOSFET gate driver could not be enabled
 >  * 45 - stop position deprecated - an attempt was made to use the deprecated "stop position" feature along with velocity or acceleration limits. Prefer to instead command the desired position directly with a target velocity of 0.0, or secondarily, disable acceleration and velocity limits.
 >  * 46 - timing violation - internal checks are enabled, and the controller violated an internal timing constraint
 >  * 47 - bemf feedforward no accel - servo.bemf_feedforward is configured, but no acceleration limit was specified. If you really know what you are doing, you can disable this with servo.bemf_feedforward_override.
 >  * 48 - invalid limits - servopos.position_min or servopos.position_max are finite and outside the available position range
 > 
 > Some non-zero codes can be presented during valid control modes without a fault. These indicate which, if any, function is limiting the output power of the controller.
 > 
 >  * 96 - servo.max_velocity
 >  * 97 - servo.max_power_W
 >  * 98 - the maximum system voltage
 >  * 99 - servo.max_current_A
 >  * 100 - servo.fault_temperature
 >  * 101 - servo.motor_fault_temperature
 >  * 102 - the commanded maximum torque
 >  * 103 - servopos.position_min or servopos.position_max

# Custom fastfetch icon/config
`~/.config/fastfetch/config.jsonc`
```jsonc
{
  "$schema": "https://github.com/fastfetch-cli/fastfetch/raw/master/doc/json_schema.json",
  "logo": {
    "source": "/home/quad/TURTLE.txt",
    "color": {
      "1": "yellow",
      "2": "white"
    }
  },
  "display": {
    "color": {
      "keys": "yellow",
      "title": "red"
    }
  },
  "modules": [
    "title",
    "separator",
    "os",
    "host",
    "kernel",
    "uptime",
    "packages",
    "shell",
    "display",
    "de",
    "wm",
    "wmtheme",
    "theme",
    "icons",
    "font",
    "cursor",
    "terminal",
    "terminalfont",
    "cpu",
    "gpu",
    "memory",
    "swap",
    "disk",
    "localip",
    "battery",
    "poweradapter",
    "locale",
    "break",
    "colors"
  ]
}
```

`~/TURTLE.txt`
```
$2       _________
       '0@@@@@@@@@P"+__          __ggp  
           "4@@@@@$1/@@_$2@@p  $1_g_$2,@@@@@@@'
           $1__  $2'<B$1@@@@$2[@@/$1@@@W$2@@@@@@@@
          $1'@@@@  _o@@@@@g@@@@$2_@@@@@@P
           $1'@@@@@@@@@@@@@@@@@$2"@P$1_~gg
             @@@@@@@@@@@@@@@@@@@@@@F    
      ,~g@g_@@@@@@@"$2g@@g$1"@@@@@@@F$2_g_
      $1`%@@@@@@@@@@$2g@@@@@@g$1@@@@@@ $2@@@@
  __g@@@@@@$1'@@@@@@$2@@@@@@@@$1g@@@@@@@@g_$24
,@@@@@@@@@D$1_@@@@@@b$2<@@@@P$1g@@@@@@B@@@B"$2g
'@@@@@@@$1_@@@@@@@@@@@@@@@@@@@@@@  $2\\@@@@@
   ""=> $1@@BD" "@@@@@@@@@@@@@@@@g, $2\\@@@@|
               $1g@@@@@@@@@B> '@@@@  $2T@@@@
              $1@@@@$2,@@g$1@@@g     "=   $2\\@@@
              $18@"$2g@@@@$1T@@P           $2'@@
               @@@@@@@ $1'"
               $2@@@@@@?
               \\@@@@/                   

```