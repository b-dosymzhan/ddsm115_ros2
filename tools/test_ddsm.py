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

"""Tests of ddsm.py against the Waveshare examples and a simulated RS485 bus."""

import ddsm
import pytest


class FakeBus:
    """
    DDSM115 motors on one RS485 bus, behind the part of the pyserial API that ddsm.py uses.

    Like the real hardware, every drive command gets a reply, replies arrive in small chunks and
    a new ID is taken after 5 set-ID frames when only one motor is connected.
    """

    def __init__(self, motor_ids, echo=False, chunk=3):
        self.motors = {motor_id: {'mode': 2, 'rpm': 0} for motor_id in motor_ids}
        self.echo = echo
        self.chunk = chunk
        self.rx = bytearray()
        self.set_id_frames = 0
        self.written = []

    def write(self, frame):
        frame = bytes(frame)
        self.written.append(frame)
        if self.echo:
            self.rx += frame
        motor = self.motors.get(frame[0])
        if frame[1] == ddsm.CMD_SET_MODE:
            if motor:
                motor['mode'] = frame[9]
            return
        if ddsm.crc8_maxim(frame[:9]) != frame[9]:
            return
        if frame[:3] == b'\xAA\x55\x53' and len(self.motors) == 1:
            self.set_id_frames += 1
            if self.set_id_frames == ddsm.SET_ID_REPEATS:
                self.motors = {frame[3]: next(iter(self.motors.values()))}
        elif frame[:2] == b'\xC8\x64' and len(self.motors) == 1:
            motor_id, motor = next(iter(self.motors.items()))
            self.rx += self.drive_reply(motor_id, motor)
        elif motor and frame[1] == ddsm.CMD_DRIVE:
            motor['rpm'] = ddsm.s16(frame[2], frame[3])
            self.rx += self.drive_reply(frame[0], motor)
        elif motor and frame[1] == ddsm.CMD_QUERY:
            rpm = motor['rpm'] & 0xFFFF
            self.rx += ddsm.with_crc(
                [frame[0], motor['mode'], 0, 0, rpm >> 8, rpm & 0xFF, 25, 64, 0])

    @staticmethod
    def drive_reply(motor_id, motor):
        rpm = motor['rpm'] & 0xFFFF
        return ddsm.with_crc([motor_id, motor['mode'], 0, 0, rpm >> 8, rpm & 0xFF, 0x40, 0, 0])

    def flush(self):
        pass

    @property
    def in_waiting(self):
        return min(len(self.rx), self.chunk)

    def read(self, size):
        count = min(size, self.chunk, len(self.rx))
        data = bytes(self.rx[:count])
        del self.rx[:count]
        return data

    def reset_input_buffer(self):
        self.rx.clear()


def frame(text):
    return bytes.fromhex(text)


def test_crc_check_value():
    assert ddsm.crc8_maxim(b'123456789') == 0xA1


@pytest.mark.parametrize('built, expected', [
    # Examples of the Waveshare DDSM115 wiki
    (ddsm.speed_frame(1, 100), '01 64 00 64 00 00 00 00 00 4F'),
    (ddsm.speed_frame(1, 50), '01 64 00 32 00 00 00 00 00 D3'),
    (ddsm.speed_frame(1, 0), '01 64 00 00 00 00 00 00 00 50'),
    (ddsm.drive_frame(1, 2000), '01 64 07 D0 00 00 00 00 00 27'),
    (ddsm.drive_frame(1, 20000), '01 64 4E 20 00 00 00 00 00 5E'),
    (ddsm.drive_frame(1, 0, brake=True), '01 64 00 00 00 00 00 FF 00 D1'),
    (ddsm.query_frame(1), '01 74 00 00 00 00 00 00 00 04'),
    (ddsm.set_id_frame(1), 'AA 55 53 01 00 00 00 00 00 CB'),
    (ddsm.id_query_frame(), 'C8 64 00 00 00 00 00 00 00 DE'),
    (ddsm.set_mode_frame(1, ddsm.MODES['velocity']), '01 A0 00 00 00 00 00 00 00 02'),
    # Negative speed is two's complement
    (ddsm.speed_frame(3, -100), '03 64 FF 9C 00 00 00 00 00 CB'),
])
def test_frames_match_known_bytes(built, expected):
    assert built == frame(expected)


def test_speed_is_clamped():
    assert ddsm.speed_frame(1, 1000) == ddsm.speed_frame(1, ddsm.RPM_LIMIT)


def test_parse_query_reply():
    reply = ddsm.parse_query_reply(frame('02 02 00 05 FF 9C 1E 80 08 55'))
    assert reply['id'] == 2
    assert reply['rpm'] == -100
    assert reply['temperature'] == 30
    assert reply['angle'] == pytest.approx(180.0)
    assert ddsm.error_text(reply['error']) == 'STALL'


def test_parse_drive_reply_has_16_bit_position():
    reply = ddsm.parse_drive_reply(ddsm.with_crc([1, 2, 0, 0, 0, 100, 0x20, 0, 0]))
    assert reply['rpm'] == 100
    assert reply['angle'] == pytest.approx(90.0, abs=0.01)


def test_error_text():
    assert ddsm.error_text(0) == 'OK'
    assert ddsm.error_text(0x0A) == 'STALL+OVERCURRENT'


def test_find_reply_skips_echo_garbage_and_other_motors():
    sent = ddsm.query_frame(2)
    other = ddsm.with_crc([3, 2, 0, 0, 0, 0, 25, 0, 0])
    wanted = ddsm.with_crc([2, 2, 0, 0, 0, 0, 25, 0, 0])
    data = b'\x00\x13' + sent + other + wanted
    assert ddsm.find_reply(data, sent, motor_id=2) == wanted
    assert ddsm.find_reply(data[:-1], sent, motor_id=2) is None


@pytest.mark.parametrize('echo', [False, True])
def test_exchange_reassembles_chunked_replies(echo):
    bus = ddsm.Bus(FakeBus([1], echo=echo, chunk=2))
    reply = bus.exchange(ddsm.speed_frame(1, 42), 1)
    assert ddsm.parse_drive_reply(reply)['rpm'] == 42


def test_exchange_ignores_stale_replies():
    port = FakeBus([1])
    bus = ddsm.Bus(port)
    bus.send(ddsm.speed_frame(1, 99))  # its reply is never read
    port.motors[1]['rpm'] = 10
    reply = bus.exchange(ddsm.query_frame(1), 1)
    assert ddsm.parse_query_reply(reply)['rpm'] == 10


def test_exchange_times_out_without_motor():
    assert ddsm.Bus(FakeBus([])).exchange(ddsm.query_frame(1), 1) is None


def test_get_and_set_id(monkeypatch):
    monkeypatch.setattr(ddsm.time, 'sleep', lambda seconds: None)
    port = FakeBus([1])
    bus = ddsm.Bus(port)
    assert ddsm.get_id(bus) == 1
    assert ddsm.set_id(bus, 4) == 4
    assert port.set_id_frames == ddsm.SET_ID_REPEATS


def test_scan_finds_every_motor():
    bus = ddsm.Bus(FakeBus([1, 3, 4]))
    assert [reply['id'] for reply in ddsm.scan(bus, range(1, 6))] == [1, 3, 4]


def test_stop_sends_zero_speed():
    port = FakeBus([1, 2])
    port.motors[1]['rpm'] = port.motors[2]['rpm'] = 50
    ddsm.stop(ddsm.Bus(port), [1, 2])
    assert port.motors[1]['rpm'] == port.motors[2]['rpm'] == 0


def test_parse_id_range():
    assert ddsm.parse_id_range('1-4') == [1, 2, 3, 4]
    assert ddsm.parse_id_range('7') == [7]


@pytest.mark.parametrize('args, expected', [
    (['frame', 'speed', '1', '100'], '01 64 00 64 00 00 00 00 00 4F'),
    (['frame', 'set-id', '02'], 'AA 55 53 02 00 00 00 00 00 92'),
    (['frame', 'mode', '4', 'velocity'], '04 A0 00 00 00 00 00 00 00 02'),
    (['frame', 'get-id'], 'C8 64 00 00 00 00 00 00 00 DE'),
])
def test_frame_command(capsys, args, expected):
    ddsm.main(args)
    assert capsys.readouterr().out.strip() == expected
