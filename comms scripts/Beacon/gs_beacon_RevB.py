#!/usr/bin/env python3
import json, socket, time, os, subprocess

PORT         = int(os.environ.get("BEACON_PORT", "9000"))
VIDEO_PORT   = int(os.environ.get("VIDEO_PORT", "5004"))
SERVICE      = os.environ.get("SERVICE_NAME", "GS-H264")
BCAST_ADDR   = os.environ.get("BROADCAST_IP", "255.255.255.255")
TICK_SEC     = float(os.environ.get("BEACON_PERIOD_SEC", "2"))
PREFERRED_IF = os.environ.get("PREFERRED_IF", "")   # e.g. "wlan0" or "eth0"
STARTUP_WAIT = float(os.environ.get("STARTUP_GRACE_SEC", "0"))  # optional

# Linux-specific constant (safe no-op elsewhere if not used)
SO_BINDTODEVICE = 25

def ip_of_iface(ifname: str) -> str | None:
    """Return IPv4 address of a specific interface (Linux)."""
    if not ifname:
        return None
    try:
        out = subprocess.check_output(
            ["bash","-lc", f"ip -4 -o addr show dev {ifname} | awk '{{print $4}}' | cut -d/ -f1"],
            text=True
        ).strip()
        return out or None
    except Exception:
        return None

def default_route_ip() -> str | None:
    """Classic UDP connect trick; returns None if no default route yet."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("1.1.1.1", 80))
            return s.getsockname()[0]
        finally:
            s.close()
    except OSError:
        return None

def hostname_scan_ip() -> str | None:
    """Fallback: parse `hostname -I` and pick a plausible IPv4."""
    try:
        out = subprocess.check_output(["hostname", "-I"], text=True).strip().split()
        for ip in out:
            if (ip.count(".") == 3
                and not ip.startswith(("127.", "169.254.", "172.17."))):
                return ip
    except Exception:
        pass
    return None

def get_local_ip() -> str | None:
    """Prefer interface IP; fallback to default-route; last resort hostname -I."""
    ip = ip_of_iface(PREFERRED_IF)
    if ip:
        return ip
    ip = default_route_ip()
    if ip:
        return ip
    return hostname_scan_ip()

def main():
    if STARTUP_WAIT > 0:
        time.sleep(STARTUP_WAIT)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    # Bind egress to a specific interface if requested (Linux only)
    if PREFERRED_IF:
        try:
            sock.setsockopt(socket.SOL_SOCKET, SO_BINDTODEVICE, PREFERRED_IF.encode() + b"\0")
        except Exception:
            # Not fatal; continue without binding
            pass

    while True:
        ip = get_local_ip()
        payload = {
            "service": SERVICE,
            "ip": ip or "0.0.0.0",   # keep field for humans; receiver should trust source IP
            "port": VIDEO_PORT,
            "proto": "rtp-h264",
            "ts": int(time.time())
        }
        msg = (json.dumps(payload) + "\n").encode()
        print(f"[beacon] broadcasting {payload} → {BCAST_ADDR}:{PORT}", flush=True)

        try:
            sock.sendto(msg, (BCAST_ADDR, PORT))
        except Exception as e:
            # Don’t crash on transient network errors; just log and retry next tick
            print(f"[beacon] send error: {e}", flush=True)

        time.sleep(TICK_SEC)

if __name__ == "__main__":
    main()
