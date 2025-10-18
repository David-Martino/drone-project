#!/usr/bin/env python3
import argparse
import socket
import struct
import time

# -------- Protocol layout (decoded frame = body + crc) --------
# body: [type:u8][counter:u32][imu_ts:u64][payload...]
HDR_FMT  = "<B I Q"
HDR_SIZE = struct.calcsize(HDR_FMT)
CRC_SIZE = 2

# Packet types (must match ESP firmware)
RAW_IMU, PROC_IMU, KALMAN = 0, 1, 2

# Expected payload sizes by type (bytes) — tweak if your structs change
EXPECTED_PAYLOAD = {
    RAW_IMU:  2*3*2,              # accel(3)*i16 + gyro(3)*i16 = 12
    PROC_IMU: 4*3*2,              # accel(3)*f32 + gyro(3)*f32 = 24
    KALMAN:   (4+12) + (4+12),    # u32+3*f32 + u32+3*f32 = 32
}

TYPE_NAME = {
    RAW_IMU:  "rawImuData",
    PROC_IMU: "processedImuData",
    KALMAN:   "kalmanStateData",
}

# -------- CRC16/CCITT-FALSE (poly 0x1021, init 0xFFFF, refin/out=false, xorout 0x0000) --------
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

# -------- Counter helpers (u32 with wraparound) --------
U32 = 0xFFFFFFFF

def next_u32(x: int) -> int:
    return (x + 1) & U32

def u32_distance(prev: int, cur: int) -> int:
    """Distance moving forward mod 2^32 from prev to cur."""
    return (cur - prev) & U32

# -------- Pretty helpers --------
def hexdump(b: bytes, max_len: int = 32) -> str:
    s = b[:max_len].hex()
    if len(b) > max_len:
        s += f"...(+{len(b)-max_len}B)"
    return s

# -------- Main receive loop --------
def run(bind_ip: str, bind_port: int, verbose: bool, dump_payload: bool):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((bind_ip, bind_port))
    sock.settimeout(1.0)

    last_counter = None
    total_ok = total_crc_bad = total_len_bad = total_ooo = total_lost = total_dup = 0
    t0 = time.time()

    print(f"Listening on UDP {bind_ip}:{bind_port}")

    while True:
        try:
            data, addr = sock.recvfrom(65535)  # 1 datagram = 1 decoded frame (body+crc)
        except socket.timeout:
            # periodic stats
            dt = time.time() - t0
            if dt >= 5.0:
                print(f"[stats] ok={total_ok} crc_bad={total_crc_bad} len_bad={total_len_bad} "
                      f"ooo={total_ooo} dup={total_dup} lost={total_lost}")
                t0 = time.time()
            continue
        except KeyboardInterrupt:
            break

        # Basic length check
        if len(data) < HDR_SIZE + CRC_SIZE:
            total_len_bad += 1
            print(f"[WARN] from {addr}: too short ({len(data)} B)")
            continue

        body = data[:-CRC_SIZE]
        rx_crc = struct.unpack("<H", data[-CRC_SIZE:])[0]
        calc_crc = crc16_ccitt_false(body)
        if rx_crc != calc_crc:
            total_crc_bad += 1
            print(f"[WARN] from {addr}: CRC mismatch (rx=0x{rx_crc:04x}, calc=0x{calc_crc:04x}) len={len(data)}")
            continue

        # Parse header (we're not unpacking payload bytes; just peeking type)
        pkt_type, counter, imu_ts = struct.unpack(HDR_FMT, body[:HDR_SIZE])
        payload = body[HDR_SIZE:]

        # Sanity check payload length by type (optional)
        expected = EXPECTED_PAYLOAD.get(pkt_type)
        if expected is not None and len(payload) != expected:
            total_len_bad += 1
            print(f"[WARN] type={pkt_type} ({TYPE_NAME.get(pkt_type,'?')}): "
                  f"payload_len={len(payload)} != expected={expected}")
            continue

        # Counter tracking (drops / duplicates / out-of-order)
        if last_counter is None:
            # first packet
            pass
        else:
            dist = u32_distance(last_counter, counter)
            if dist == 0:
                total_dup += 1
                print(f"[WARN] duplicate counter={counter}")
            elif dist == 1:
                # in-order
                pass
            elif dist <= (1 << 31):
                # jumped forward: lost (dist-1) packets
                lost = dist - 1
                total_lost += lost
                print(f"[WARN] lost {lost} packet(s): {last_counter} -> {counter}")
            else:
                # wrapped backwards or out-of-order arrival
                total_ooo += 1
                print(f"[WARN] out-of-order: {last_counter} -> {counter}")

        last_counter = counter
        total_ok += 1

        # Print valid packet summary
        name = TYPE_NAME.get(pkt_type, f"unknown({pkt_type})")
        if dump_payload:
            print(f"[OK] {name} cnt={counter} ts_us={imu_ts} len={len(payload)} payload={hexdump(payload)}")
        else:
            print(f"[OK] {name} cnt={counter} ts_us={imu_ts} len={len(payload)}")

def main():
    ap = argparse.ArgumentParser(description="IMU UDP receiver (CRC + counter checks)")
    ap.add_argument("--bind", default="0.0.0.0", help="Bind IP (default: 0.0.0.0)")
    ap.add_argument("--port", type=int, default=56000, help="UDP port (default: 56000)")
    ap.add_argument("-v", "--verbose", action="store_true", help="Verbose logging")
    ap.add_argument("--dump-payload", action="store_true", help="Hexdump payload bytes")
    args = ap.parse_args()
    run(args.bind, args.port, args.verbose, args.dump_payload)

if __name__ == "__main__":
    main()
