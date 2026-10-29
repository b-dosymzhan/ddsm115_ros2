# Troubleshooting

These are the problems we ran into while building a 4-wheel robot on DDSM115 motors.

## No motor answers

1. **Port and permission.** Run `ls -l /dev/serial/by-id/`, and `sudo usermod -aG dialout $USER`
   (then log out and back in).
2. **Power.** The motors need their own 12–24 V supply. USB powers only the adapter.
3. **A and B swapped.** Swap them and try again. It is harmless and very common.
4. **No common ground.** Connect the adapter GND to the motor supply GND.
5. **Unknown ID.** Connect one motor and run `ddsm.py get-id`. Then `ddsm.py scan` tries every ID.

## Replies have bad CRCs, or two motors look like one

Two motors have the **same ID** and answer at the same time. Connect them one at a time and give
each its own ID with `ddsm.py set-id`.

## `set-id` does not change the ID

- More than one motor is connected. Only one may be on the bus.
- An ID was already set since the motor was powered on. It works once per power-up, so
  power-cycle the motor and try again.

## Feedback is sometimes wrong or belongs to another request

**Every drive command (0x64) gets a reply**, not only the feedback query. If you send speed commands
and then a query, the first 10 bytes you read are the reply to the speed command. Before we flushed
the input buffer (`tcflush` in C, `reset_input_buffer()` in pyserial) before each query, about 15 %
of the replies were parsed wrongly. Both the tool and the driver do this flush now.

## Replies arrive in pieces

USB-RS485 adapters often deliver a 10-byte reply in chunks of 2–3 bytes. Keep reading until you
have 10 bytes, or until a timeout of a few milliseconds. A single `read()` right after the request
usually returns only part of the reply.

## The angle or temperature looks like nonsense

The two replies use bytes 6 and 7 differently. In the feedback query reply (0x74), byte 6 is the
winding temperature and byte 7 is an 8-bit position. Only the reply to a drive command has a 16-bit
position. See [protocol.md](protocol.md#replies).

## The mode frame "works" but is wrong

The mode frame `ID A0 00 00 00 00 00 00 00 mode` has **no CRC**: the last byte is the mode. Our
first scripts appended a CRC there (`01 A0 … 9E`), which is not a valid mode. It went unnoticed
because the motors start in velocity mode anyway. It breaks as soon as you need current or
position mode.

## One side of the robot drives backward

Mirrored motors. Set `-1` for that side in `motor_directions`
(see [ros2_control.md](ros2_control.md#1-directions)).

## The robot does not drive straight

Use `motor_trims` to slow down the faster side, for example `0.98`.

## The robot turns more or less than the odometry says

The effective wheel separation of a skid-steer robot is not the measured track. Calibrate it with
the spin-in-place test in [ros2_control.md](ros2_control.md#3-wheel-separation-skid-steer).

## It worked yesterday, today the port is wrong

`/dev/ttyUSB*` numbers follow the plug-in order. Use `/dev/serial/by-id/...` in your launch files.

## The robot cannot reach the speed I command

The protocol accepts 330 rpm, but the motor's speed depends on the supply voltage (about 11 rpm
per volt). At 14.8 V expect about 160 rpm without load. See
[protocol.md](protocol.md#speed-limits).
