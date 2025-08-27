# MoveIt Servo Demo – `robot_fanuc`

This package provides a MoveIt Servo demo for controlling a Fanuc robot using either a joystick or keyboard input. Below are the basic usage instructions.

## Launch the MoveIt Servo Example

```bash
ros2 launch robot_fanuc servo_example.launch.py
```

## Start Servo via Service Call

After launching, start the Servo functionality by calling the service:

```bash
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
```

## Keyboard Control (No Controller Needed)

To control the robot using keyboard input (without a joystick/controller):

```bash
ros2 run robot_fanuc servo_keyboard_input
```

## Joystick Data

To view incoming joystick data (for debugging or setup):

```bash
ros2 topic echo /joy
```

## Notes

- Make sure your workspace is built and sourced:
  ```bash
  colcon build --packages-select robot_fanuc
  source install/setup.bash
  ```
- Ensure your joystick is connected and recognized by the system if you intend to use joystick control.

For more details, see the package source or launch files.


## udev rule

get device info with: udevadm info -a -n /dev/input/js0
Example output:
```
  looking at parent device '/devices/pci0000:00/0000:00:14.0/usb1/1-7/1-7:1.0/input/input28':
    KERNELS=="input28"
    SUBSYSTEMS=="input"
    DRIVERS==""
    ATTRS{capabilities/abs}=="3003f"
    ATTRS{capabilities/ev}=="20000b"
    ATTRS{capabilities/ff}=="107030000 0"
    ATTRS{capabilities/key}=="7cdb000000000000 0 0 0 0"
    ATTRS{capabilities/led}=="0"
    ATTRS{capabilities/msc}=="0"
    ATTRS{capabilities/rel}=="0"
    ATTRS{capabilities/snd}=="0"
    ATTRS{capabilities/sw}=="0"
    ATTRS{id/bustype}=="0003"
    ATTRS{id/product}=="02ea"
    ATTRS{id/vendor}=="045e"
    ATTRS{id/version}=="0301"
    ATTRS{inhibited}=="0"
    ATTRS{name}=="Microsoft X-Box One S pad"
    ATTRS{phys}=="usb-0000:00:14.0-7/input0"
    ATTRS{power/async}=="disabled"
    ATTRS{power/control}=="auto"
    ATTRS{power/runtime_active_kids}=="0"
    ATTRS{power/runtime_active_time}=="0"
    ATTRS{power/runtime_enabled}=="disabled"
    ATTRS{power/runtime_status}=="unsupported"
    ATTRS{power/runtime_suspended_time}=="0"
    ATTRS{power/runtime_usage}=="0"
    ATTRS{properties}=="0"
    ATTRS{uniq}==""
```

/etc/udev/rules.d/99-joystick.rules
```bash
SUBSYSTEM=="input", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="02ea", SYMLINK+="robot_mindset_joystick"
SUBSYSTEM=="input", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="02ea", SYMLINK+="robot_mindset_joystick_events"
```

sudo udevadm control --reload