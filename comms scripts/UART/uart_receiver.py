#!/usr/bin/env python3
import sys, time, struct, binascii, argparse
from collections import deque
from typing import Iterator

try:
    import serial  # pip install pyserial
except ImportError:
    print("Please: pip install pyserial", file=sys.stderr); sys.exit(1)

# ---------- Protocol assumptions ----------
# IMU frame header: <B I Q> + payload + crc16  (COBS + 0x00)
HDR_FMT  = "<B I Q"
HDR_SIZE = struct.calcsize(HDR_FMT)
CRC_SIZE = 2

SYNC_TYPE = 0xFF  # 255, reserved by firmware for UART sync response
SYNC_BODY_SIZE = 1 + 8 + 8 + 2  # type(1) + t1(8) + t2(8) + crc(2)

# ---------- COBS ----------
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

# ---------- CRC16/CCITT-FALSE ----------
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

# ---------- Helpers ----------
def now_us() -> int:
    return int(time.time_ns() // 1000)

# ---------- Rolling stats ----------
class WindowStats:
    def __init__(self, n=100):
        self.samples = deque(maxlen=n)  # (offset_us, rtt_us)
    def push(self, off, rtt):
        self.samples.append((off, rtt))
    def summary(self):
        if not self.samples:
            return None
        offs = [x[0] for x in self.samples]
        rtts = [x[1] for x in self.samples]
        def _avg(a): return sum(a)/len(a) if a else 0.0
        def _p(a, p):
            if not a: return 0.0
            s = sorted(a); k = max(0, min(len(s)-1, int(round((p/100)*(len(s)-1)))))
            return s[k]
        return dict(
            n=len(self.samples),
            off=dict(avg=_avg(offs), p50=_p(offs,50), p90=_p(offs,90), p99=_p(offs,99), mn=min(offs), mx=max(offs)),
            rtt=dict(avg=_avg(rtts), p50=_p(rtts,50), p90=_p(rtts,90), p99=_p(rtts,99), mn=min(rtts), mx=max(rtts)),
        )

# ---------- Frame parsing ----------
def try_parse_sync(dec: bytes):
    """Return (t1_dev, t2_dev) if this is exactly the UART_SYNC response frame, else None."""
    if len(dec) != SYNC_BODY_SIZE:
        return None
    body = dec[:-CRC_SIZE]
    rx   = struct.unpack("<H", dec[-CRC_SIZE:])[0]
    if crc16_ccitt_false(body) != rx:
        return None
    if dec[0] != SYNC_TYPE:
        return None
    t1, t2 = struct.unpack_from("<QQ", body, 1)
    return (t1, t2)

def try_parse_imu(dec: bytes):
    """Return (typ, cnt, imu_ts, payload) if looks like IMU frame with header, else None."""
    if len(dec) < HDR_SIZE + CRC_SIZE:
        return None
    body = dec[:-CRC_SIZE]
    rx   = struct.unpack("<H", dec[-CRC_SIZE:])[0]
    if crc16_ccitt_false(body) != rx:
        return None
    try:
        typ, cnt, imu_ts = struct.unpack_from(HDR_FMT, body, 0)
    except struct.error:
        return None
    payload = body[HDR_SIZE:]
    return (typ, cnt, imu_ts, payload)

# ---------- Main loop ----------
def main():
    ap = argparse.ArgumentParser(description="Trigger UART sync via single 0x00 after every N IMU frames; compute offset/RTT")
    ap.add_argument("--port", default="/dev/serial0", help="serial device")
    ap.add_argument("--baud", type=int, default=1000000, help="baud rate")
    ap.add_argument("--sync-freq", type=int, default=1000, help="send sync after every N IMU frames (N>=20)")
    ap.add_argument("--resp-window", type=int, default=5, help="scan next N frames for sync response")
    ap.add_argument("--quiet", action="store_true", help="less verbose per-IMU output")
    ap.add_argument("--stats-window", type=int, default=100, help="rolling stats window")
    args = ap.parse_args()

    # Enforce minimum sync frequency
    if args.sync_freq < 20:
        print(f"Warning: --sync-freq {args.sync_freq} too low, clamping to 20", file=sys.stderr)
        args.sync_freq = 20

    try:
        ser = serial.Serial(
            args.port, args.baud, timeout=0.002,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            xonxoff=False, rtscts=False, dsrdtr=False, exclusive=True,
        )
    except Exception as e:
        print(f"Failed to open {args.port}@{args.baud}: {e}", file=sys.stderr)
        sys.exit(2)

    print(f"Listening on {args.port}@{args.baud} … sync every {args.sync_freq} IMU frames, resp-window={args.resp_window}. Ctrl-C to stop")

    buf = bytearray()
    imu_count = 0

    # Sync tracking
    pending = False
    frames_left = 0
    t0_host = 0
    hits = 0
    misses = 0

    stats = WindowStats(args.stats_window)
    last_stats = time.time()

    try:
        while True:
            chunk = ser.read(4096)
            if chunk:
                buf.extend(chunk)

            for enc in split_frames(buf):
                if not enc:
                    continue
                try:
                    dec = cobs_decode(enc)
                except Exception:
                    continue  # ignore corrupt frames

                # If a sync is pending, check this frame first
                if pending:
                    t3 = now_us()
                    parsed_sync = try_parse_sync(dec)
                    if parsed_sync is not None:
                        t1, t2 = parsed_sync
                        rtt = (t3 - t0_host) - (t2 - t1)
                        off = ((t1 - t0_host) + (t2 - t3)) / 2.0
                        stats.push(off, rtt)
                        hits += 1
                        print(f"[sync] HIT  off={off:+.1f}us  rtt={rtt:.1f}us  t0={t0_host} t1={t1} t2={t2} t3={t3}")
                        pending = False
                        # Don’t consume the frame further; it's a sync frame
                        continue
                    else:
                        # Not a sync → decrement frames_left and continue to normal parsing
                        frames_left -= 1
                        if frames_left <= 0:
                            misses += 1
                            print("[sync] MISS (no sync found in window)")
                            pending = False
                        # fall-through: treat this frame normally

                # Normal handling
                parsed_imu = try_parse_imu(dec)
                if parsed_imu is not None:
                    typ, cnt, imu_ts, payload = parsed_imu
                    imu_count += 1
                    if not args.quiet:
                        print(f"[imu] type={typ} cnt={cnt} imu_ts={imu_ts} paylen={len(payload)}")
                    # Time to trigger sync?
                    if (imu_count % args.sync_freq) == 0 and not pending:
                        t0_host = now_us()
                        try:
                            ser.write(b"\x00")  # one falling edge → ISR
                        except Exception as e:
                            misses += 1
                            print(f"[sync] MISS (write failed: {e})")
                        else:
                            pending = True
                            frames_left = max(1, args.resp_window)
                    continue

                # Unknown/other packets: ignore quietly

            # Periodic stats every ~5s
            if (time.time() - last_stats) > 5.0:
                s = stats.summary()
                if s:
                    o, r = s['off'], s['rtt']
                    print(f"[stats] sync hits={hits} misses={misses} (win {s['n']})  "
                          f"off(us) avg={o['avg']:.1f} p50={o['p50']:.1f} p90={o['p90']:.1f} "
                          f"p99={o['p99']:.1f} min={o['mn']:.1f} max={o['mx']:.1f}  "
                          f"rtt(us) avg={r['avg']:.1f} p50={r['p50']:.1f} p90={r['p90']:.1f} "
                          f"p99={r['p99']:.1f} min={r['mn']:.1f} max={r['mx']:.1f}")
                last_stats = time.time()

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        s = stats.summary()
        print("\n---- Summary ----")
        print(f"sync hits : {hits}")
        print(f"sync miss : {misses}")
        if s:
            o, r = s['off'], s['rtt']
            print(f"offset(us): avg={o['avg']:.1f} p50={o['p50']:.1f} p90={o['p90']:.1f} "
                  f"p99={o['p99']:.1f} min={o['mn']:.1f} max={o['mx']:.1f}")
            print(f"rtt(us)   : avg={r['avg']:.1f} p50={r['p50']:.1f} p90={r['p90']:.1f} "
                  f"p99={r['p99']:.1f} min={r['mn']:.1f} max={r['mx']:.1f}")
        print("Done.")

if __name__ == "__main__":
    main()


# #!/usr/bin/env python3
# import sys, time, struct, binascii, argparse
# from collections import deque
# from typing import Iterator, Optional

# try:
#     import serial  # pip install pyserial
# except ImportError:
#     print("Please: pip install pyserial", file=sys.stderr); sys.exit(1)

# # ---------- Protocol assumptions ----------
# # IMU frame header: <B I Q> + payload + crc16  (COBS + 0x00)
# HDR_FMT  = "<B I Q"
# HDR_SIZE = struct.calcsize(HDR_FMT)
# CRC_SIZE = 2

# SYNC_TYPE = 0xFF  # 255, reserved by firmware for UART sync response
# SYNC_BODY_SIZE = 1 + 8 + 8 + 2  # type(1) + t1(8) + t2(8) + crc(2)

# # ---------- COBS ----------
# def cobs_decode(data: bytes) -> bytes:
#     out = bytearray()
#     i, n = 0, len(data)
#     while i < n:
#         code = data[i]
#         if code == 0:
#             raise ValueError("COBS: zero code inside frame")
#         i += 1
#         cnt = code - 1
#         if i + cnt > n:
#             raise ValueError("COBS: block overruns input")
#         out += data[i:i+cnt]
#         i += cnt
#         if code < 0xFF and i < n:
#             out.append(0)
#     return bytes(out)

# def split_frames(buf: bytearray) -> Iterator[bytes]:
#     """Yield COBS-encoded frames (without trailing 0x00)."""
#     start = 0
#     while True:
#         try:
#             idx = buf.index(0x00, start)
#         except ValueError:
#             if start > 0:
#                 del buf[:start]
#             return
#         yield bytes(buf[start:idx])
#         start = idx + 1

# # ---------- CRC16/CCITT-FALSE ----------
# def crc16_ccitt_false(data: bytes) -> int:
#     crc = 0xFFFF
#     for b in data:
#         crc ^= (b << 8) & 0xFFFF
#         for _ in range(8):
#             if crc & 0x8000:
#                 crc = ((crc << 1) ^ 0x1021) & 0xFFFF
#             else:
#                 crc = (crc << 1) & 0xFFFF
#     return crc

# # ---------- Helpers ----------
# def now_us() -> int:
#     return int(time.time_ns() // 1000)

# def fmt_hex(b: bytes, n=64) -> str:
#     h = binascii.hexlify(b).decode("ascii")
#     return h if len(h) <= n else h[:n] + "…"

# # ---------- Rolling stats ----------
# class WindowStats:
#     def __init__(self, n=100):
#         self.n = n
#         self.samples = deque(maxlen=n)  # (offset_us, rtt_us)
#     def push(self, off, rtt):
#         self.samples.append((off, rtt))
#     def summary(self):
#         if not self.samples:
#             return None
#         offs = [x[0] for x in self.samples]
#         rtts = [x[1] for x in self.samples]
#         def _avg(a): return sum(a)/len(a) if a else 0.0
#         def _p(a, p):
#             if not a: return 0.0
#             s = sorted(a); k = max(0, min(len(s)-1, int(round((p/100)*(len(s)-1)))))
#             return s[k]
#         return dict(
#             n=len(self.samples),
#             off=dict(avg=_avg(offs), p50=_p(offs,50), p90=_p(offs,90), p99=_p(offs,99), mn=min(offs), mx=max(offs)),
#             rtt=dict(avg=_avg(rtts), p50=_p(rtts,50), p90=_p(rtts,90), p99=_p(rtts,99), mn=min(rtts), mx=max(rtts)),
#         )

# # ---------- Frame parsing ----------
# def try_parse_sync(dec: bytes):
#     """Return (t1_dev, t2_dev) if this is exactly the UART_SYNC response frame, else None."""
#     if len(dec) != SYNC_BODY_SIZE:
#         return None
#     # CRC check
#     body = dec[:-CRC_SIZE]
#     rx   = struct.unpack("<H", dec[-CRC_SIZE:])[0]
#     if crc16_ccitt_false(body) != rx:
#         return None
#     typ = dec[0]
#     if typ != SYNC_TYPE:
#         return None
#     t1, t2 = struct.unpack_from("<QQ", body, 1)
#     return (t1, t2)

# def try_parse_imu(dec: bytes):
#     """Return (typ, cnt, imu_ts, payload) if looks like IMU frame with header, else None."""
#     if len(dec) < HDR_SIZE + CRC_SIZE:
#         return None
#     body = dec[:-CRC_SIZE]
#     rx   = struct.unpack("<H", dec[-CRC_SIZE:])[0]
#     if crc16_ccitt_false(body) != rx:
#         return None
#     try:
#         typ, cnt, imu_ts = struct.unpack_from(HDR_FMT, body, 0)
#     except struct.error:
#         return None
#     payload = body[HDR_SIZE:]
#     return (typ, cnt, imu_ts, payload)

# # ---------- Main loop ----------
# def main():
#     ap = argparse.ArgumentParser(description="Trigger UART sync via single 0x00 after every N IMU frames; compute offset/RTT")
#     ap.add_argument("--port", default="/dev/serial0", help="serial device")
#     ap.add_argument("--baud", type=int, default=1000000, help="baud rate")
#     ap.add_argument("--sync-freq", type=int, default=1000, help="send sync after every N IMU frames (N>=1)")
#     ap.add_argument("--resp-timeout-us", type=int, default=3000, help="timeout waiting for *next* frame after request")
#     ap.add_argument("--quiet", action="store_true", help="less verbose per-IMU output")
#     ap.add_argument("--stats-window", type=int, default=100, help="rolling stats window")
#     args = ap.parse_args()

#     if args.sync_freq < 1:
#         print("--sync-freq must be >= 1", file=sys.stderr); sys.exit(2)

#     try:
#         ser = serial.Serial(
#             args.port, args.baud, timeout=0.002,  # short read timeout
#             bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
#             xonxoff=False, rtscts=False, dsrdtr=False, exclusive=True,
#         )
#     except Exception as e:
#         print(f"Failed to open {args.port}@{args.baud}: {e}", file=sys.stderr)
#         sys.exit(2)

#     print(f"Listening on {args.port}@{args.baud} … sync every {args.sync_freq} IMU frames. Ctrl-C to stop")

#     buf = bytearray()
#     imu_count = 0

#     pending = False
#     t0_host = 0
#     misses  = 0
#     hits    = 0

#     stats = WindowStats(args.stats_window)
#     start = time.time()

#     try:
#         while True:
#             # Read some bytes
#             chunk = ser.read(4096)
#             if chunk:
#                 buf.extend(chunk)

#             # Extract frames
#             for enc in split_frames(buf):
#                 if not enc:
#                     continue
#                 # Decode COBS — accept errors silently (diagnostic prints removed for cleanliness)
#                 try:
#                     dec = cobs_decode(enc)
#                 except Exception:
#                     continue

#                 # If we just sent a sync request, the **very next** frame decides hit/miss
#                 if pending:
#                     t3 = now_us()
#                     parsed_sync = try_parse_sync(dec)
#                     if parsed_sync is not None:
#                         t1, t2 = parsed_sync
#                         # NTP style
#                         rtt = (t3 - t0_host) - (t2 - t1)
#                         off = ((t1 - t0_host) + (t2 - t3)) / 2.0
#                         stats.push(off, rtt)
#                         hits += 1
#                         print(f"[sync] HIT  off={off:+.1f}us  rtt={rtt:.1f}us  t0={t0_host} t1={t1} t2={t2} t3={t3}")
#                         pending = False
#                         continue  # done with this frame

#                     # Not a sync response → treat normally (don’t drop), but count a miss
#                     misses += 1
#                     print(f"[sync] MISS (next frame was not SYNC)")
#                     pending = False
#                     # Fall through to regular handling of this frame

#                 # Normal handling: IMU or other packets
#                 parsed_imu = try_parse_imu(dec)
#                 if parsed_imu is not None:
#                     typ, cnt, imu_ts, payload = parsed_imu
#                     imu_count += 1
#                     if not args.quiet:
#                         # Keep IMU logging light
#                         print(f"[imu] type={typ} cnt={cnt} imu_ts={imu_ts} paylen={len(payload)}")
#                     # Time to trigger sync?
#                     if imu_count % args.sync_freq == 0 and not pending:
#                         # Send a single 0x00 byte to create one falling edge (start bit)
#                         t0_host = now_us()
#                         ser.write(b"\x00")
#                         pending = True
#                         # We only consider the *very next* frame; if nothing arrives quickly,
#                         # the first subsequent frame (whenever it comes) will be miss/hit judged above.
#                     continue

#                 # Could be other packet types; ignore quietly
#                 # (If you want to debug unknown frames, print length/first bytes here.)

#             # If we’re pending but nothing has arrived for a while, also treat as miss
#             if pending:
#                 # Check coarse timeout based on wall clock
#                 # (We mark miss when next frame finally arrives; this is just a guard
#                 #  in case the line goes idle for a long time.)
#                 # Not strictly necessary; we rely on “next frame decides”.
#                 pass

#             # Periodic stats (every ~5s of wall time)
#             if (time.time() - start) > 5.0:
#                 s = stats.summary()
#                 if s:
#                     o, r = s['off'], s['rtt']
#                     print(f"[stats] sync hits={hits} misses={misses} (win {s['n']})  "
#                           f"off(us) avg={o['avg']:.1f} p50={o['p50']:.1f} p90={o['p90']:.1f} "
#                           f"p99={o['p99']:.1f} min={o['mn']:.1f} max={o['mx']:.1f}  "
#                           f"rtt(us) avg={r['avg']:.1f} p50={r['p50']:.1f} p90={r['p90']:.1f} "
#                           f"p99={r['p99']:.1f} min={r['mn']:.1f} max={r['mx']:.1f}")
#                 start = time.time()

#     except KeyboardInterrupt:
#         pass
#     finally:
#         ser.close()
#         s = stats.summary()
#         print("\n---- Summary ----")
#         print(f"sync hits : {hits}")
#         print(f"sync miss : {misses}")
#         if s:
#             o, r = s['off'], s['rtt']
#             print(f"offset(us): avg={o['avg']:.1f} p50={o['p50']:.1f} p90={o['p90']:.1f} "
#                   f"p99={o['p99']:.1f} min={o['mn']:.1f} max={o['mx']:.1f}")
#             print(f"rtt(us)   : avg={r['avg']:.1f} p50={r['p50']:.1f} p90={r['p90']:.1f} "
#                   f"p99={r['p99']:.1f} min={r['mn']:.1f} max={r['mx']:.1f}")
#         print("Done.")

# if __name__ == "__main__":
#     main()
