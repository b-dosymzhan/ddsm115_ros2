#!/usr/bin/env python3
# Copyright 2026 Dosymzhan Birzhanuly
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Set up and test Waveshare DDSM115 hub motors over RS485, without ROS.

    ddsm.py --port /dev/ttyUSB0 get-id                  # only one motor on the bus
    ddsm.py --port /dev/ttyUSB0 set-id 3                # only one motor on the bus
    ddsm.py --port /dev/ttyUSB0 scan                    # list the motors on the bus
    ddsm.py --port /dev/ttyUSB0 spin --ids 1 2 --rpm 60 # w / s / q / Esc
    ddsm.py --port /dev/ttyUSB0 stop --ids 1 2 3 4
    ddsm.py --port /dev/ttyUSB0 send 01 74 00 00 00 00 00 00 00   # CRC is appended
    ddsm.py frame speed 1 100                           # print a frame, no port needed

Works on Linux, macOS and Windows (--port COM5). Needs pyserial.
"""

import argparse
import os
import select
import sys
import time

BAUD = 115200
FRAME_SIZE = 10

CMD_DRIVE = 0x64
CMD_QUERY = 0x74
CMD_SET_MODE = 0xA0
COMMAND_CODES = (CMD_DRIVE, CMD_QUERY, CMD_SET_MODE)

MODES = {'current': 0x01, 'velocity': 0x02, 'position': 0x03}
RPM_LIMIT = 330
AMPS_FULL_SCALE = 8.0      # raw current 32767 is 8 A

REPLY_TIMEOUT = 0.02       # s, a motor answers about 1.5 ms after the request
SET_ID_REPEATS = 5         # the motor takes a new ID after 5 identical frames
SET_ID_GAP = 0.05          # s between the set-ID frames
FRAME_GAP = 0.02           # s between mode frames, which get no reply
SPIN_PERIOD = 0.05         # s, speed commands are resent every cycle


# ---------------------------------------------------------------------------------------------
# Frames

def crc8_maxim(data: bytes) -> int:
    """CRC-8/MAXIM (reflected polynomial 0x8C, initial value 0) of the DDSM protocol."""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 1 else crc >> 1
    return crc


def with_crc(body) -> bytes:
    """Append the CRC to the first 9 bytes of a frame."""
    body = bytes(body)
    return body + bytes([crc8_maxim(body)])


def drive_frame(motor_id: int, value: int, accel_time: int = 0, brake: bool = False) -> bytes:
    """
    Build a drive command; its meaning depends on the motor mode.

    velocity: signed rpm (-330..330), current: signed raw current (-32767..32767 is -8..8 A),
    position: 0..32767 over one revolution. accel_time (velocity mode) is the ramp in
    0.1 ms per rpm, 0 uses the motor default. brake (velocity mode) stops the wheel hard.
    """
    value &= 0xFFFF
    return with_crc([motor_id, CMD_DRIVE, value >> 8, value & 0xFF, 0, 0,
                     accel_time & 0xFF, 0xFF if brake else 0, 0])


def speed_frame(motor_id: int, rpm: int, accel_time: int = 0) -> bytes:
    """Velocity command, clamped to the motor limit."""
    return drive_frame(motor_id, max(-RPM_LIMIT, min(RPM_LIMIT, rpm)), accel_time)


def query_frame(motor_id: int) -> bytes:
    """Ask a motor for speed, current, temperature, position and faults."""
    return with_crc([motor_id, CMD_QUERY, 0, 0, 0, 0, 0, 0, 0])


def set_mode_frame(motor_id: int, mode: int) -> bytes:
    """Select the control loop. This frame has no CRC: the last byte is the mode."""
    return bytes([motor_id, CMD_SET_MODE, 0, 0, 0, 0, 0, 0, 0, mode])


def set_id_frame(new_id: int) -> bytes:
    """Give a new ID to the only motor on the bus (send it 5 times)."""
    return with_crc([0xAA, 0x55, 0x53, new_id, 0, 0, 0, 0, 0])


def id_query_frame() -> bytes:
    """Ask the only motor on the bus for its ID."""
    return with_crc([0xC8, 0x64, 0, 0, 0, 0, 0, 0, 0])


# ---------------------------------------------------------------------------------------------
# Replies

def s16(high: int, low: int) -> int:
    """Signed 16-bit value from two bytes."""
    value = (high << 8) | low
    return value - 0x10000 if value & 0x8000 else value


def error_text(error: int) -> str:
    """Fault flags as text, for example 'STALL+OVERCURRENT'."""
    flags = [name for bit, name in ((3, 'STALL'), (2, 'PHASE_OVERCURRENT'),
                                    (1, 'OVERCURRENT'), (0, 'SENSOR')) if error & (1 << bit)]
    return '+'.join(flags) if flags else 'OK'


def parse_drive_reply(frame: bytes) -> dict:
    """Reply to a drive command or to the ID query: the position has 16 bits."""
    return {
        'id': frame[0], 'mode': frame[1],
        'current': s16(frame[2], frame[3]) * AMPS_FULL_SCALE / 32767,
        'rpm': s16(frame[4], frame[5]),
        'angle': ((frame[6] << 8) | frame[7]) * 360.0 / 32767,
        'error': frame[8],
    }


def parse_query_reply(frame: bytes) -> dict:
    """Reply to a query frame (0x74): temperature and an 8-bit position."""
    return {
        'id': frame[0], 'mode': frame[1],
        'current': s16(frame[2], frame[3]) * AMPS_FULL_SCALE / 32767,
        'rpm': s16(frame[4], frame[5]),
        'temperature': frame[6],
        'angle': frame[7] * 360.0 / 256,
        'error': frame[8],
    }


def find_reply(data: bytes, sent: bytes, motor_id=None):
    """
    Find a motor reply in received bytes, or None.

    Skips the echo of our own frame (some adapters echo), anything with a command code in byte 1,
    frames with a wrong CRC and replies from another motor.
    """
    for start in range(len(data) - FRAME_SIZE + 1):
        frame = bytes(data[start:start + FRAME_SIZE])
        if frame == sent or frame[1] in COMMAND_CODES or crc8_maxim(frame[:9]) != frame[9]:
            continue
        if motor_id is not None and frame[0] != motor_id:
            continue
        return frame
    return None


# ---------------------------------------------------------------------------------------------
# Bus

class Bus:
    """Request / reply exchange on the RS485 bus; `port` is a pyserial port or a test double."""

    def __init__(self, port):
        """Wrap an open port."""
        self.port = port

    def send(self, frame: bytes):
        """Send a frame without waiting for a reply."""
        self.port.write(frame)
        self.port.flush()

    def exchange(self, frame: bytes, motor_id=None, timeout: float = REPLY_TIMEOUT):
        """Send a frame and return the reply, or None after `timeout` seconds."""
        # Motors answer every drive command too; drop those replies so they are not mistaken
        # for the answer to this frame
        self.port.reset_input_buffer()
        self.send(frame)
        received = bytearray()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            # USB-RS485 adapters often deliver a reply in chunks of 2-3 bytes
            received += self.port.read(self.port.in_waiting or 1)
            reply = find_reply(received, frame, motor_id)
            if reply:
                return reply
        return None


def get_id(bus: Bus):
    """ID of the only motor on the bus, or None."""
    reply = bus.exchange(id_query_frame(), timeout=0.1)
    return reply[0] if reply else None


def set_id(bus: Bus, new_id: int):
    """Give `new_id` to the only motor on the bus; returns the ID it reports afterwards."""
    for _ in range(SET_ID_REPEATS):
        bus.send(set_id_frame(new_id))
        time.sleep(SET_ID_GAP)
    time.sleep(0.5)  # let the motor store the ID
    return get_id(bus)


def scan(bus: Bus, ids):
    """Query every ID and return the decoded replies of the motors that answered."""
    found = []
    for motor_id in ids:
        reply = bus.exchange(query_frame(motor_id), motor_id)
        if reply:
            found.append(parse_query_reply(reply))
    return found


def set_mode(bus: Bus, ids, mode: int):
    """Switch the control loop of the motors (no reply)."""
    for motor_id in ids:
        bus.send(set_mode_frame(motor_id, mode))
        time.sleep(FRAME_GAP)


def stop(bus: Bus, ids):
    """Send zero speed to the motors."""
    for motor_id in ids:
        bus.exchange(speed_frame(motor_id, 0), motor_id)


# ---------------------------------------------------------------------------------------------
# Interactive spin test

class Keyboard:
    """Non-blocking single-key input on Linux, macOS and Windows."""

    def __enter__(self):
        """Switch the terminal to single-key input."""
        if os.name != 'nt':
            import termios
            import tty
            self.fd = sys.stdin.fileno()
            self.saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)  # keys without Enter, Ctrl+C still works
        return self

    def __exit__(self, *args):
        """Restore the terminal."""
        if os.name != 'nt':
            import termios
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)

    def key(self):
        """Return the pressed key in lower case, or None."""
        if os.name == 'nt':
            import msvcrt
            return msvcrt.getwch().lower() if msvcrt.kbhit() else None
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1).lower()
        return None


def status_line(motor_id: int, command: int, drive, query) -> str:
    """Format the replies of one motor for the spin screen."""
    if not drive and not query:
        return f'motor {motor_id:3d} | cmd {command:4d} rpm | (no reply)'
    fb = parse_drive_reply(drive) if drive else parse_query_reply(query)
    temperature = f'{query[6]:3d} C' if query else '  ? C'
    return (f'motor {motor_id:3d} | cmd {command:4d} rpm | speed {fb["rpm"]:4d} rpm | '
            f'angle {fb["angle"]:6.1f} deg | current {fb["current"]:5.2f} A | '
            f'temp {temperature} | {error_text(fb["error"])}')


def spin(bus: Bus, ids, directions, rpm: int):
    """Drive the motors from the keyboard and show their feedback until Esc."""
    set_mode(bus, ids, MODES['velocity'])
    target = 0
    with Keyboard() as keyboard:
        while True:
            started = time.monotonic()
            key = keyboard.key()
            if key == '\x1b':
                return
            if key in ('w', 's', 'q', ' '):
                target = {'w': rpm, 's': -rpm}.get(key, 0)

            lines = [f'target {target} rpm | w forward, s reverse, q or space stop, Esc exit', '']
            for motor_id, direction in zip(ids, directions):
                command = target * direction
                drive = bus.exchange(speed_frame(motor_id, command), motor_id)
                query = bus.exchange(query_frame(motor_id), motor_id)
                lines.append(status_line(motor_id, command, drive, query))
            print('\033[2J\033[H' + '\n'.join(lines), flush=True)

            time.sleep(max(0.0, SPIN_PERIOD - (time.monotonic() - started)))


# ---------------------------------------------------------------------------------------------
# Command line

def hex_text(frame: bytes) -> str:
    """Format bytes as '01 64 00 ...'."""
    return ' '.join(f'{byte:02X}' for byte in frame)


def number(text: str) -> int:
    """Decimal or 0x hex; '01' is accepted as 1."""
    try:
        return int(text, 0)
    except ValueError:
        return int(text, 10)


FRAME_HELP = """frame kinds:
  speed ID RPM              velocity command, -330..330 rpm
  current ID MILLIAMPS      current command, -8000..8000 mA
  position ID DEGREES       position command, 0..360
  brake ID                  stop hard (velocity mode)
  query ID                  ask for feedback
  mode ID velocity|current|position
  set-id NEW_ID             send it 5 times, one motor on the bus
  get-id                    one motor on the bus"""


def make_frame(kind: str, values):
    """Build a frame from the `frame` command arguments."""
    def need(count):
        if len(values) != count:
            sys.exit(f'"frame {kind}" needs {count} value(s)\n\n{FRAME_HELP}')

    if kind == 'mode':
        need(2)
        if values[1] not in MODES:
            sys.exit(f'mode must be one of {", ".join(MODES)}')
        return set_mode_frame(number(values[0]), MODES[values[1]])
    if kind == 'get-id':
        need(0)
        return id_query_frame()

    builders = {
        'speed': (2, lambda i, rpm: speed_frame(i, rpm)),
        'current': (2, lambda i, ma: drive_frame(i, round(ma / 1000 / AMPS_FULL_SCALE * 32767))),
        'position': (2, lambda i, deg: drive_frame(i, round((deg % 360) / 360 * 32767))),
        'brake': (1, lambda i: drive_frame(i, 0, brake=True)),
        'query': (1, query_frame),
        'set-id': (1, set_id_frame),
    }
    if kind not in builders:
        sys.exit(f'unknown frame "{kind}"\n\n{FRAME_HELP}')
    count, build = builders[kind]
    need(count)
    return build(*[number(value) for value in values])


def parse_id_range(text: str):
    """'1-4' or '7' -> list of IDs."""
    first, _, last = text.partition('-')
    return list(range(int(first), int(last or first) + 1))


def open_port(name: str):
    """Open the adapter at 115200 baud, or exit with a hint."""
    try:
        import serial
    except ImportError:
        sys.exit('pyserial is missing: pip install pyserial (or sudo apt install python3-serial)')
    try:
        return serial.Serial(name, BAUD, timeout=0.002)
    except serial.SerialException as error:
        hint = ''
        if 'ermission' in str(error):
            hint = '\nAdd yourself to the dialout group: sudo usermod -aG dialout $USER, ' \
                   'then log out and back in.'
        elif os.name != 'nt':
            hint = '\nList the adapters with: ls -l /dev/serial/by-id/'
        sys.exit(f'Cannot open {name}: {error}{hint}')


def build_parser():
    """Command line of the tool."""
    parser = argparse.ArgumentParser(
        description='Set up and test Waveshare DDSM115 hub motors over RS485.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split('\n\n')[1])
    default_port = 'COM3' if os.name == 'nt' else '/dev/ttyUSB0'
    parser.add_argument('--port', default=default_port,
                        help=f'USB-RS485 adapter (default {default_port})')
    commands = parser.add_subparsers(dest='command', required=True)

    commands.add_parser('get-id', help='read the ID of the only motor on the bus')

    p = commands.add_parser('set-id', help='give a new ID to the only motor on the bus')
    p.add_argument('new_id', type=int, help='1..253')

    p = commands.add_parser('scan', help='list the motors that answer')
    p.add_argument('--range', default='1-253', help='IDs to try, for example 1-10')

    p = commands.add_parser('spin', help='spin motors with the keyboard and show feedback')
    p.add_argument('--ids', type=int, nargs='+', default=[1, 2, 3, 4])
    p.add_argument('--rpm', type=int, default=60, help='test speed (max 330)')
    p.add_argument('--directions', type=int, nargs='+',
                   help='1 or -1 per motor, so that w drives every wheel forward')

    p = commands.add_parser('stop', help='send zero speed')
    p.add_argument('--ids', type=int, nargs='+', default=[1, 2, 3, 4])

    p = commands.add_parser('mode', help='switch the control loop of motors')
    p.add_argument('mode', choices=list(MODES))
    p.add_argument('--ids', type=int, nargs='+', default=[1, 2, 3, 4])

    p = commands.add_parser('send', help='send a raw frame and print the reply')
    p.add_argument('bytes', nargs='+', help='9 hex bytes (CRC is appended) or 10 hex bytes')

    p = commands.add_parser('frame', help='print a frame (no port needed)', epilog=FRAME_HELP,
                            formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('kind')
    p.add_argument('values', nargs='*')
    return parser


def main(argv=None):
    """Run one command."""
    args = build_parser().parse_args(argv)

    if args.command == 'frame':
        print(hex_text(make_frame(args.kind, args.values)))
        return

    port = open_port(args.port)
    bus = Bus(port)
    try:
        if args.command == 'get-id':
            motor_id = get_id(bus)
            print(f'motor ID: {motor_id}' if motor_id is not None else
                  'No answer. Check the wiring and that exactly one motor is connected.')

        elif args.command == 'set-id':
            if not 1 <= args.new_id <= 253:
                sys.exit('the ID must be 1..253')
            print(f'Setting ID {args.new_id}. Only ONE motor may be on the bus.')
            motor_id = set_id(bus, args.new_id)
            if motor_id == args.new_id:
                print(f'Done, the motor now has ID {motor_id}.')
            elif motor_id is None:
                print('No answer to the ID query. Check the wiring and the port.')
            else:
                print(f'The motor still reports ID {motor_id}. An ID can be set only once per '
                      'power-up: power-cycle the motor and try again.')

        elif args.command == 'scan':
            found = scan(bus, parse_id_range(args.range))
            for fb in found:
                print(f'ID {fb["id"]:3d} | mode {fb["mode"]} | {fb["rpm"]:4d} rpm | '
                      f'{fb["temperature"]} C | {error_text(fb["error"])}')
            print(f'{len(found)} motor(s) found.')

        elif args.command == 'spin':
            directions = args.directions or [1] * len(args.ids)
            if len(directions) != len(args.ids):
                sys.exit('--directions needs one value per ID')
            print('Lift the wheels off the ground. Starting in 2 s...')
            time.sleep(2)
            try:
                spin(bus, args.ids, directions, args.rpm)
            except KeyboardInterrupt:
                pass
            finally:
                stop(bus, args.ids)
                print('Motors stopped.')

        elif args.command == 'stop':
            stop(bus, args.ids)

        elif args.command == 'mode':
            set_mode(bus, args.ids, MODES[args.mode])
            print(f'{args.mode} mode sent. Switching to position mode needs the wheel '
                  'below 10 rpm.' if args.mode == 'position' else f'{args.mode} mode sent.')

        elif args.command == 'send':
            data = bytes.fromhex(''.join(args.bytes))
            if len(data) == FRAME_SIZE - 1:
                data = with_crc(data)
            elif len(data) != FRAME_SIZE:
                sys.exit('give 9 or 10 bytes')
            print('TX', hex_text(data))
            reply = bus.exchange(data, timeout=0.1)
            print('RX', hex_text(reply) if reply else '(no reply)')
    finally:
        port.close()


if __name__ == '__main__':
    main()
