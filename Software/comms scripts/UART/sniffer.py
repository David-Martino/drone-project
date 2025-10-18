#!/usr/bin/env python3
import sys, time, struct, binascii, argparse
from typing import Iterator

try:
    import serial  # pip install pyserial
except ImportError:
    print("Please: pip install pyserial", file=sys.stderr); sys.exit(1)

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
def split_frames(buf: bytearray) -> Iterator[bytes]:
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

def fmt_hex(b: bytes) -> str:
    return binascii.hexlify(b).decode("ascii")

def now_us() -> int:
    return int(time.time_ns() // 1000)

def main():
    ap = argparse.ArgumentParser(description="COBS+CRC IMU UART sniffer (print + summary)")
    ap.add_argument("--port", default="/dev/serial0", help="serial device")
    ap.add_argument("--baud", type=int, default=1000000, help="baud rate")
    ap.add_argument("--read-chunk", type=int, default=4096, help="serial read chunk size")
    ap.add_argument("--throttle", type=int, default=0, help="print every Nth OK packet (0 = print all)")
    args = ap.parse_args()

    ser = serial.Serial(
        args.port, args.baud, timeout=0.01,
        bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
        xonxoff=False, rtscts=False, dsrdtr=False, exclusive=True,
    )

    buf = bytearray()
    HDR_FMT  = "<B I Q"
    HDR_SIZE = struct.calcsize(HDR_FMT)
    CRC_SIZE = 2

    # Counters
    start_t = time.time()
    pkt_total = 0          # decoded frames (good + bad-CRC + bad-len/hdr)
    good = 0
    cobs_err = 0
    crc_err = 0
    len_err = 0
    hdr_err = 0
    printed_ok = 0

    print(f"Listening on {args.port}@{args.baud} … Ctrl-C to stop")
    try:
        while True:
            chunk = ser.read(args.read_chunk)
            if not chunk:
                continue
            buf.extend(chunk)

            for enc in split_frames(buf):
                if not enc:
                    continue

                # Decode COBS
                try:
                    dec = cobs_decode(enc)  # dec = body + crc16
                except Exception as e:
                    cobs_err += 1
                    print(f"[{now_us()}] COBS-DECODE-ERR: {e} enc={fmt_hex(enc)}")
                    continue

                if len(dec) < HDR_SIZE + CRC_SIZE:
                    len_err += 1
                    print(f"[{now_us()}] LEN-ERR: len(dec)={len(dec)} < {HDR_SIZE+CRC_SIZE} dec={fmt_hex(dec)}")
                    continue

                body   = dec[:-CRC_SIZE]
                rx_crc = struct.unpack("<H", dec[-CRC_SIZE:])[0]
                calc   = crc16_ccitt_false(body)

                # Parse header
                try:
                    typ, cnt, ts = struct.unpack(HDR_FMT, body[:HDR_SIZE])
                except struct.error as e:
                    hdr_err += 1
                    print(f"[{now_us()}] HDR-UNPACK-ERR: {e} body={fmt_hex(body)}")
                    continue

                payload = body[HDR_SIZE:]
                pkt_total += 1

                if rx_crc != calc:
                    crc_err += 1
                    print(
                        f"[{now_us()}] BAD-CRC type={typ} cnt={cnt} imu_ts={ts} "
                        f"payload_len={len(payload)} payload={fmt_hex(payload)} "
                        f"crc_rx=0x{rx_crc:04x} crc_calc=0x{calc:04x}"
                    )
                    continue

                # OK packet
                good += 1
                # Throttle applies only to OK packets; errors always print.
                if args.throttle and (good % args.throttle) != 0:
                    continue

                printed_ok += 1
                print(
                    f"[{now_us()}] OK type={typ} cnt={cnt} imu_ts={ts} "
                    f"payload_len={len(payload)} payload={fmt_hex(payload)} "
                    f"crc=0x{rx_crc:04x}"
                )

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        dur = max(1e-9, time.time() - start_t)
        total_frames = good + crc_err + len_err + hdr_err + cobs_err  # everything we classified
        err_frames = total_frames - good
        err_rate = (err_frames / total_frames * 100.0) if total_frames else 0.0
        print("\n---- Summary ----")
        print(f"duration        : {dur:.2f}s")
        print(f"decoded total   : {total_frames}")
        print(f"  good          : {good}")
        print(f"  cobs_err      : {cobs_err}")
        print(f"  len_err       : {len_err}")
        print(f"  hdr_err       : {hdr_err}")
        print(f"  crc_err       : {crc_err}")
        print(f"printed OK pkts : {printed_ok} (throttle={args.throttle})")
        print(f"error rate      : {err_rate:.3f}%")

if __name__ == "__main__":
    main()


# import serial, struct, time

# PORT="/dev/serial0"; BAUD=921600; READ=4096
# def cobs_decode(d: bytes)->bytes:
#     o=bytearray(); i=0; n=len(d)
#     while i<n:
#         code=d[i]; 
#         if code==0: raise ValueError("zero code")
#         i+=1; cnt=code-1
#         if i+cnt>n: raise ValueError("overrun")
#         o+=d[i:i+cnt]; i+=cnt
#         if code<0xFF and i<n: o.append(0)
#     return bytes(o)
# def crc16_ccitt_false(b: bytes)->int:
#     c=0xFFFF
#     for x in b:
#         c^=(x<<8)&0xFFFF
#         for _ in range(8):
#             c=((c<<1)^0x1021)&0xFFFF if c&0x8000 else (c<<1)&0xFFFF
#     return c
# HDR_FMT="<B I Q"; HDR_SIZE=struct.calcsize(HDR_FMT); CRC_SIZE=2

# ser=serial.Serial(PORT, BAUD, timeout=0.02)
# buf=bytearray(); good=badc=badk=badl=0
# print(f"Listening on {PORT}@{BAUD}…")
# try:
#     while True:
#         buf.extend(ser.read(READ))
#         start=0
#         while True:
#             try: idx=buf.index(0x00, start)
#             except ValueError:
#                 if start>0: del buf[:start]
#                 break
#             enc=bytes(buf[start:idx]); start=idx+1
#             if not enc: continue
#             try: dec=cobs_decode(enc)
#             except ValueError: badc+=1; continue
#             if len(dec)<HDR_SIZE+CRC_SIZE: badl+=1; continue
#             body=dec[:-CRC_SIZE]; rx=struct.unpack("<H", dec[-CRC_SIZE:])[0]
#             if crc16_ccitt_false(body)!=rx: badk+=1; continue
#             typ,cnt,ts=struct.unpack_from(HDR_FMT, body, 0)
#             payload=body[HDR_SIZE:]
#             good+=1
#             print(f"ok type={typ} cnt={cnt} ts={ts} paylen={len(payload)} first8={payload[:8].hex()}")
# except KeyboardInterrupt:
#     pass
# finally:
#     ser.close()
#     print(f"good={good} cobs_err={badc} crc_err={badk} len_err={badl}")
