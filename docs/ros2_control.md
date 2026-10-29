# ros2_control driver

`ddsm_hardware/DDSMHardwareInterface` is a ros2_control `SystemInterface`. It drives any number of
DDSM115 motors on one RS485 bus. Every joint is one motor:

- **Command:** `velocity` in rad/s.
- **States:** `velocity` in rad/s, measured by the motor, and `position` in rad, integrated from
  the velocity.

## Parameters

| Parameter | Default | |
|---|---|---|
| `port` | required | serial device of the USB-RS485 adapter |
| `motor_ids` | required | comma separated, one per joint, in joint order |
| `motor_directions` | required | `1` or `-1` per motor, so that a positive joint velocity drives forward |
| `motor_trims` | all `1.0` | speed factor per motor, to make the robot drive straight |
| `rpm_limit` | `330` | commands are clamped to ± this value |
| `accel_time` | `0` | ramp sent with every command, 0.1 ms per rpm; 0 = motor default |
| `brake` | `0` | brake byte sent with every command; keep 0 |

The joints are listed inside the `<ros2_control>` tag, and their order maps onto the lists. With
`motor_ids` `1,2,3,4` and joints `front_left, rear_left, front_right, rear_right`, motor 1 is the
front left wheel.

## Lifecycle

| Step | What the driver does |
|---|---|
| configure | opens the port at 115200 baud |
| activate | puts every motor in velocity mode |
| `write()` | sends one speed command per motor |
| `read()` | queries every motor and waits for its reply |
| deactivate | sends zero speed to every motor |
| cleanup | closes the port |

When a motor does not answer, `read()` keeps its last value, moves on to the next motor and logs a
throttled warning. Fault flags (stall, overcurrent, …) are logged the same way.

## Update rate

Each `read()` takes about 3 ms per motor: the query, the 1.5 ms the motor needs, and its reply.
Our 4-wheel robot ran the controller manager at **20 Hz**. Fewer motors leave room for a higher
rate.

## Calibration

### 1. Directions

Left and right motors are mirrored. Run `tools/ddsm.py spin` with the wheels in the air, then find
the `--directions` that turn every wheel forward. Use them as `motor_directions`.

In ROS, check it with a small forward command:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

### 2. Straight-line trim

Two motors never run exactly alike. Drive 5 m straight and see which way the robot drifts. Then
slow down the faster side with `motor_trims`. On our robot the right side runs at `0.98`.

### 3. Wheel separation (skid-steer)

A 4-wheel skid-steer robot slips sideways when it turns, so its **effective** wheel separation is
not the measured track. Calibrate it:

1. Mark the robot's heading on the floor.
2. Turn in place until `/odom` says the robot turned 360°, slowly, for example
   `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"`.
3. Measure the angle the robot really turned.
4. Set `wheel_separation_new = wheel_separation × odom_angle / real_angle`.

If the robot really turned 300° while the odometry said 360°, the new value is 1.2 times the old
one. Repeat until both agree. The calibrated value depends on the floor.

The odometry of a skid-steer robot stays poor while it turns in place. Fusing the gyro of an IMU
with [robot_localization](https://github.com/cra-ros-pkg/robot_localization) helps a lot.

## Tips

- `diff_drive_controller` stops the robot when no command arrives for `cmd_vel_timeout` (0.5 s in
  the example).
- `Could not enable FIFO RT scheduling policy` from the controller manager is harmless.
- `ros2 control list_hardware_interfaces` shows whether the joints are claimed.
- Run `ros2_control_node` with `output='both'` to see the driver's warnings.
