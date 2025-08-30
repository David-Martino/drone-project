#!/usr/bin/env python3
import json, socket, sys, os, time

PORT = int(os.environ.get("BEACON_PORT", "9000"))
TIMEOUT = float(os.environ.get("DISCOVERY_TIMEOUT", "30"))
SERVICE = os.environ.get("SERVICE_NAME", "GS-H264")

def main():
    deadline = time.time() + TIMEOUT
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", PORT))
    sock.settimeout(1.0)

    last = None
    consecutive = 0
    need_consecutive = 2   # require 2 matching beacons

    while time.time() < deadline:
        try:
            data, addr = sock.recvfrom(2048)
            j = json.loads(data.decode(errors="ignore"))
            if j.get("service") != SERVICE:
                continue
            ip = j.get("ip")
            port = int(j.get("port", 0))
            if not ip or not port:
                continue
            cur = (ip, port)
            if cur == last:
                consecutive += 1
            else:
                last = cur
                consecutive = 1
            if consecutive >= need_consecutive:
                print(f"{ip}:{port}")
                return 0
        except socket.timeout:
            pass
        except Exception:
            pass
    return 1

if __name__ == "__main__":
    sys.exit(main())


#/usr/local/bin/discover_gs.py
#Save on pi to extract IP
#TEST: BEACON_PORT=9000 /usr/local/bin/discover_gs.py && echo OK || echo TIMEOUT
