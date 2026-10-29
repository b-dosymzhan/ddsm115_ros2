# DDSM115 RS485 protocol

This summarizes the [Waveshare DDSM115 wiki](https://www.waveshare.com/wiki/DDSM115) and what we
measured on the motors. Every example frame on this page is checked by the unit tests
(`tools/test_ddsm.py`, `ddsm_hardware/test/test_ddsm_protocol.cpp`).

## Bus

- RS485, half duplex, **115200 baud, 8 data bits, no parity, 1 stop bit**.
- Every command and every reply has **10 bytes**.
- The computer asks and one motor answers, so wait for the reply before the next request.
  A motor answers about **1.5 ms** after a request.
- **Byte 9 is a CRC-8/MAXIM** (reflected polynomial 0x8C, initial value 0) of bytes 0–8. The mode
  frame is the only exception.

```python
def crc8_maxim(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 1 else crc >> 1
    return crc
```

```c
uint8_t crc8_maxim(const uint8_t *data, size_t size) {
  uint8_t crc = 0;
  for (size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) crc = (crc & 1) ? (crc >> 1) ^ 0x8C : crc >> 1;
  }
  return crc;
}
```

## Commands

### Drive (0x64)

```
ID  64  value_hi  value_lo  00  00  accel_time  brake  00  CRC
```

`value` depends on the mode of the motor:

| Mode | value | Range |
|---|---|---|
| velocity (default) | signed rpm | −330 … 330 |
| current | signed current | −32767 … 32767 = −8 … 8 A |
| position | unsigned angle | 0 … 32767 = 0 … 360° |

Negative values are two's complement: −100 rpm is `FF 9C`.

- `accel_time` applies in velocity mode. It sets the ramp in steps of 0.1 ms per rpm, so 10 means
  1 ms per rpm and 100 rpm takes 100 ms. 0 uses the motor default.
- `brake = FF` stops the wheel hard (velocity mode only). Any other value means no brake.

**Every drive command gets a reply** (see below). Read or discard it, or it will be mistaken for
the answer to your next request.

### Feedback query (0x74)

```
ID  74  00  00  00  00  00  00  00  CRC
```

### Mode (0xA0)

```
ID  A0  00  00  00  00  00  00  00  mode
```

`mode`: `01` current, `02` velocity, `03` position. **This frame has no CRC: the last byte is the
mode.** It gets no reply. Slow the wheel below 10 rpm before switching to position mode.
Motors start in velocity mode.

### Set ID

```
AA  55  53  new_id  00  00  00  00  00  CRC
```

- Only **one motor** may be connected.
- The motor takes the ID after **5** of these frames.
- It works **once per power-up**.
- The ID is kept when the power is off.

### Get ID

```
C8  64  00  00  00  00  00  00  00  DE
```

Only one motor may be connected. It answers with a drive reply whose byte 0 is its ID.

## Replies

| Byte | Reply to drive / get ID | Reply to feedback query (0x74) |
|---|---|---|
| 0 | ID | ID |
| 1 | mode (1, 2 or 3) | mode |
| 2–3 | torque current, signed (same scale as the current command) | torque current |
| 4–5 | speed, signed rpm | speed, signed rpm |
| 6 | position, high byte | **winding temperature, °C** |
| 7 | position, low byte (0 … 32767 = 0 … 360°) | **position, 0 … 255 = 0 … 360°** |
| 8 | fault flags | fault flags |
| 9 | CRC | CRC |

Bytes 6 and 7 differ between the two replies. Reading the 0x74 reply as a 16-bit position gives
nonsense angles; older versions of our own test script had this bug.

### Fault flags (byte 8)

| Bit | Fault |
|---|---|
| 0 | sensor error |
| 1 | overcurrent |
| 2 | phase overcurrent |
| 3 | stall |
| 4 | troubleshooting (listed by Waveshare) |
| 5–7 | reserved |

## Example frames

| Purpose | Frame |
|---|---|
| Motor 1, velocity mode | `01 A0 00 00 00 00 00 00 00 02` |
| Motor 1, +100 rpm | `01 64 00 64 00 00 00 00 00 4F` |
| Motor 1, +50 rpm | `01 64 00 32 00 00 00 00 00 D3` |
| Motor 3, −100 rpm | `03 64 FF 9C 00 00 00 00 00 CB` |
| Motor 1, stop | `01 64 00 00 00 00 00 00 00 50` |
| Motor 1, brake | `01 64 00 00 00 00 00 FF 00 D1` |
| Motor 1, current 2000 (≈ 0.49 A) | `01 64 07 D0 00 00 00 00 00 27` |
| Motor 1, position 20000 (≈ 220°) | `01 64 4E 20 00 00 00 00 00 5E` |
| Motor 1, feedback | `01 74 00 00 00 00 00 00 00 04` |
| Set ID 1 | `AA 55 53 01 00 00 00 00 00 CB` |
| Set ID 2 | `AA 55 53 02 00 00 00 00 00 92` |
| Set ID 4 | `AA 55 53 04 00 00 00 00 00 20` |
| Get ID | `C8 64 00 00 00 00 00 00 00 DE` |

`python3 tools/ddsm.py frame --help` builds any other frame.

## Speed limits

The protocol accepts ±330 rpm, but the datasheet gives a speed constant of about **11 rpm per
volt** and a no-load speed of 200 ± 10 rpm. The reachable speed depends on the supply: a 4S Li-Po
battery (14.8 V) gives roughly 160 rpm without load. Set `rpm_limit` and the controller velocity
limits from your supply voltage, not from 330.

With the 100.7 mm tyre, 1 rpm is about 5.3 mm/s, so 100 rpm is about 0.53 m/s.
