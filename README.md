# DDSM115 ROS 2

Set up, test and drive **Waveshare DDSM115 hub motors** over RS485, with or without ROS 2.

| | |
|---|---|
| [`tools/ddsm.py`](tools/ddsm.py) | One-file command line tool for Linux, macOS and Windows. Assign motor IDs, find the motors on the bus, spin them with live feedback, send raw frames. Needs only `pyserial`. |
| [`ddsm_hardware`](ddsm_hardware) | ros2_control hardware interface (ROS 2 Humble). Any number of motors on one bus, velocity commands, velocity and position states. |
| [`ddsm_example`](ddsm_example) | A 2- or 4-wheel robot (differential drive or skid-steer): one launch command, then drive it with the keyboard. |
| [`docs/`](docs) | The [protocol](docs/protocol.md) with checked example frames, [wiring](docs/wiring.md), [calibration](docs/ros2_control.md) and [problems we hit](docs/troubleshooting.md). |

The driver comes from a 4-wheel delivery robot that maps and navigates with Nav2.

## What you need

- DDSM115 motors and a 12–24 V supply. Different DDSM models cannot share one bus.
- A USB-RS485 adapter with automatic direction control (a CH340-based one works).
- Python 3 with `pyserial` for the tool, ROS 2 Humble for the driver.

Wiring, power and the Waveshare DDSM Driver HAT (A) are covered in [docs/wiring.md](docs/wiring.md).

## 1. Give every motor its own ID

All motors share one bus, so each needs a unique ID. **Connect one motor at a time:**

```bash
pip install pyserial          # or: sudo apt install python3-serial
python3 tools/ddsm.py --port /dev/ttyUSB0 get-id     # which ID does it have now?
python3 tools/ddsm.py --port /dev/ttyUSB0 set-id 1   # give it ID 1
```

- The motor takes a new ID only after receiving the set-ID frame 5 times. The tool sends it 5 times.
- An ID can be set **once per power-up**. To change it again, power-cycle the motor.
- The ID is kept when the power is off. Write it on the motor.

Then connect all motors and check that every ID answers:

```bash
python3 tools/ddsm.py --port /dev/ttyUSB0 scan --range 1-10
# ID   1 | mode 2 |    0 rpm | 24 C | OK
# ...
```

On Windows use `python tools\ddsm.py --port COM5 ...`. On Linux, `ls -l /dev/serial/by-id/` shows the adapter.

## 2. Spin test

Lift the wheels off the ground, then:

```bash
python3 tools/ddsm.py --port /dev/ttyUSB0 spin --ids 1 2 3 4 --rpm 60
```

`w` forward, `s` reverse, `q` stop, `Esc` exit. Every motor shows its speed, angle, current,
winding temperature and fault flags. The motors are always stopped on exit.

Motors on the left and right of a robot are mirrored, so one side turns backward. Find the
`--directions` that drive every wheel forward, for example `--directions 1 1 -1 -1`. These are the
`motor_directions` for ROS.

## 3. Drive it with ROS 2

```bash
mkdir -p ~/ddsm_ws/src && cd ~/ddsm_ws/src
git clone https://github.com/b-dosymzhan/ddsm115_ros2.git
cd ~/ddsm_ws
rosdep install --from-paths src --ignore-src -y
colcon build && source install/setup.bash

ros2 launch ddsm_example bringup.launch.py port:=/dev/ttyUSB0 wheels:=4 \
    motor_ids:=1,2,3,4 motor_directions:=1,1,-1,-1
ros2 run teleop_twist_keyboard teleop_twist_keyboard       # in a second terminal
```

| Topic | |
|---|---|
| `/cmd_vel` (`geometry_msgs/Twist`) | velocity command |
| `/odom`, TF `odom → base_link` | wheel odometry |
| `/joint_states` | wheel positions and velocities |

The motor lists follow the joint order: `front_left, rear_left, front_right, rear_right`
(4 wheels) or `left, right` (2 wheels).

### In your own robot

Add this to your URDF and list your wheel joints in the same order as the motor lists:

```xml
<ros2_control name="DDSM115" type="system">
  <hardware>
    <plugin>ddsm_hardware/DDSMHardwareInterface</plugin>
    <param name="port">/dev/ttyUSB0</param>
    <param name="motor_ids">1,2,3,4</param>
    <param name="motor_directions">1,1,-1,-1</param>
    <param name="motor_trims">1.0,1.0,1.0,1.0</param>   <!-- optional -->
    <param name="rpm_limit">330</param>                  <!-- optional -->
  </hardware>
  <joint name="front_left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
    <state_interface name="position"/>
  </joint>
  <!-- ... one block per wheel -->
</ros2_control>
```

[docs/ros2_control.md](docs/ros2_control.md) lists every parameter. It also explains how to
calibrate the directions, straight-line trims and the effective wheel separation of a skid-steer
robot.

## The protocol in 30 seconds

115200 baud, 8N1. Every frame has 10 bytes and the last byte is a CRC-8/MAXIM of the first 9.

| Frame | Bytes | Reply |
|---|---|---|
| Speed | `ID 64 rpm_hi rpm_lo 00 00 accel brake 00 CRC` | speed, current, 16-bit position, faults |
| Feedback | `ID 74 00 00 00 00 00 00 00 CRC` | speed, current, temperature, 8-bit position, faults |
| Mode | `ID A0 00 00 00 00 00 00 00 mode` (no CRC) | none |
| Set ID | `AA 55 53 ID 00 00 00 00 00 CRC`, 5 times | none |
| Get ID | `C8 64 00 00 00 00 00 00 00 DE` | like the speed reply |

`ddsm.py frame speed 1 100` prints any frame with its CRC (`01 64 00 64 00 00 00 00 00 4F`), for use
from an Arduino, ESP32 or PLC. All the details are in [docs/protocol.md](docs/protocol.md).

## Development

```bash
python3 -m pytest tools                    # the tool, against a simulated motor bus
colcon test && colcon test-result --all    # protocol gtests, URDF test, ament linters
```

## License

[Apache-2.0](LICENSE)
