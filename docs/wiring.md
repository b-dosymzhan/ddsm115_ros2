# Wiring

```
                 USB                 RS485 A/B + GND
  Computer ─────────────── USB-RS485 ─────────────────┬──────────┬──────────┬──────────┐
                           adapter                    │          │          │          │
                                                   motor 1    motor 2    motor 3    motor 4
                                                      │          │          │          │
  Battery 12–24 V ── fuse ── e-stop / relay ──────────┴──────────┴──────────┴──────────┘
                                                      motor power (V+ / GND)
```

## Motors

- Each DDSM115 cable carries the motor power and the RS485 pair. Follow the pinout in the
  [Waveshare wiki](https://www.waveshare.com/wiki/DDSM115) for your cable.
- All motors share one RS485 bus: every A to A, every B to B. Each motor needs its own ID
  (see [the README](../README.md#1-give-every-motor-its-own-id)).
- Supply 12–24 V. The motors need their own power: the USB adapter only carries the signal.
- Connect the GND of the adapter to the GND of the motor supply, so the RS485 signal has a common
  reference.
- Different DDSM models (DDSM115, DDSM210, …) use different frames and cannot share one bus.

## USB-RS485 adapter

Use an adapter that switches the transmit direction **automatically**. Most CH340 or FT232
adapters with A, B and GND terminals do. Adapters without automatic direction need the software
to toggle a pin, which neither the tool nor the driver does.

If no motor answers, swap A and B. Mislabelled adapters are common, and swapping is harmless.

## Waveshare DDSM Driver HAT (A)

The HAT takes 9–28 V on XT60 (or DC 5525) and has four motor ports. It also has an ESP32 that
controls the motors with JSON commands over USB.

On our robot the HAT only **distributes power**: its ESP32 is not used. An external USB-RS485
adapter talks to the motors on the HAT's bus, so the computer gets the raw frames and the feedback
directly. You can also skip the HAT and wire the power yourself.

## Emergency stop

Cut the **motor power**, not the signal: a stop sent over the bus needs a working computer, cable
and bus. A normally-closed emergency button in series with a relay works well. On our robot an
ESP32 drives the relay from a ROS topic, and the button cuts power without any software.

## Serial port on Linux

```bash
ls -l /dev/serial/by-id/                  # find the adapter
sudo usermod -aG dialout $USER            # permission to open it, then log out and back in
```

`/dev/ttyUSB0`, `/dev/ttyUSB1`, … are numbered in plug-in order and can change after a reboot.
Use the stable `/dev/serial/by-id/...` path in launch files, for example
`port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`.

## Serial port on Windows

The adapter shows up as `COMx` in Device Manager. CH340 adapters may need the CH340 driver.
Then `python tools\ddsm.py --port COM5 scan`.
