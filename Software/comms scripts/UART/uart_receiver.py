#!/usr/bin/env python3
import sys, struct, socket, time
from typing import Optional

try:
    import serial  # pip install pyserial
except ImportError:
    print("Please: pip install pyserial", file=sys.stderr); sys.exit(1)

# --------- Config ---------
SER_PORT = "/dev/ttyUSB0"   # change as needed
SER_BAUD = 921600
UDP_IP   = "192.168.1.50"   # ground station IP
UDP_PORT = 56000

READ_CHUNK = 4096

# Packet layout (LE): [type:u8][counter:u32][imu_ts:u64][payload...][crc:u16]
HDR_FMT   = "<B I Q"
HDR_SIZE  = struct.calcsize(HDR_FMT)
CRC_SIZE  = 2

# (Optional) expected payload sizes by type, for a quick length sanity check
RAW_IMU, PROC_IMU, KALMAN = 0, 1, 2
EXPECTED_PAYLOAD = {
    RAW_IMU:  2*3*2,               # accel(3)*i16 + gyro(3)*i16 = 12
    PROC_IMU: 4*3*2,               # accel(3)*f32 + gyro(3)*f32 = 24
    KALMAN:   (4+12) + (4+12),     # u32+3*f32 + u32+3*f32 = 32
}

# --------- COBS decode ---------
def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i, n = 0, len(data)
    while i < n:
        code = data[i]
        if code == 0:
            raise ValueError("COBS: zero code inside frame")
        i += 1
        cnt = code - 1
        if i + cnt > n:
            raise ValueError("COBS: block overruns input")
        out += data[i:i+cnt]
        i += cnt
        if code < 0xFF and i < n:
            out.append(0)
    return bytes(out)

# --------- CRC16/CCITT-FALSE ---------
def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

# --------- Helpers ---------
def split_frames(buf: bytearray):
    """Yield COBS-encoded frames (without trailing 0x00)."""
    start = 0
    while True:
        try:
            idx = buf.index(0x00, start)
        except ValueError:
            if start > 0:
                del buf[:start]
            return
        yield bytes(buf[start:idx])
        start = idx + 1

# Placeholder mapping: replace with your a·esp + b later
def map_esp_to_pi_us(_esp_ts: int) -> int:
    # For bring-up, either return a constant or actual Pi time:
    # return 1234567890
    return int(time.time_ns() // 1000)

def main():
    ser = serial.Serial(
        SER_PORT, SER_BAUD, timeout=0.01,
        bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
        xonxoff=False, rtscts=False, dsrdtr=False, exclusive=True,
    )
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    buf = bytearray()
    sent = bad_cobs = bad_crc = bad_len = 0
    print(f"UART {SER_PORT}@{SER_BAUD} → UDP {UDP_IP}:{UDP_PORT}")

    try:
        while True:
            chunk = ser.read(READ_CHUNK)
            if not chunk:
                continue
            buf.extend(chunk)

            for enc in split_frames(buf):
                if not enc:
                    continue

                # Decode COBS
                try:
                    dec = cobs_decode(enc)  # dec = body + crc
                except ValueError:
                    bad_cobs += 1
                    continue

                if len(dec) < HDR_SIZE + CRC_SIZE:
                    bad_len += 1
                    continue

                body = bytearray(dec[:-CRC_SIZE])
                rx_crc = struct.unpack("<H", dec[-CRC_SIZE:])[0]

                # Verify CRC on original body
                if crc16_ccitt_false(body) != rx_crc:
                    bad_crc += 1
                    continue

                # Optional: length sanity by type (peek type only)
                pkt_type = body[0]
                exp = EXPECTED_PAYLOAD.get(pkt_type)
                if exp is not None:
                    payload_len = len(body) - HDR_SIZE
                    if payload_len != exp:
                        bad_len += 1
                        continue

                # Overwrite imu_ts (u64 LE at offset 1+4) with Pi-mapped time
                new_pi_ts_us = map_esp_to_pi_us(0)  # we ignore esp ts for now
                struct.pack_into("<Q", body, 1 + 4, new_pi_ts_us)

                # Recompute CRC and send via UDP as a single datagram
                new_crc = crc16_ccitt_false(body)
                out = bytes(body) + struct.pack("<H", new_crc)
                try:
                    sock.sendto(out, (UDP_IP, UDP_PORT))
                    sent += 1
                except Exception as e:
                    # UDP send failed; drop this frame
                    pass

            # (Optional) lightweight stats
            # if sent % 500 == 0 and sent > 0:
            #     print(f"sent={sent} bad_cobs={bad_cobs} bad_crc={bad_crc} bad_len={bad_len}")

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        sock.close()
        print(f"sent={sent} bad_cobs={bad_cobs} bad_crc={bad_crc} bad_len={bad_len}")

if __name__ == "__main__":
    main()
