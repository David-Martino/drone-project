#!/usr/bin/env python3
import json, socket, sys, os, time
from collections import defaultdict

PORT = int(os.environ.get("BEACON_PORT", "9000"))
TIMEOUT = float(os.environ.get("DISCOVERY_TIMEOUT", "30"))
SERVICE = os.environ.get("SERVICE_NAME", "GS-H264")
NEED_CONSECUTIVE = int(os.environ.get("NEED_CONSECUTIVE", "2"))

def main():
    deadline = time.time() + TIMEOUT
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", PORT))
    sock.settimeout(1.0)

    # track consecutive counts per (ip, port)
    consec = defaultdict(int)
    last_key = None

    while time.time() < deadline:
        try:
            data, addr = sock.recvfrom(4096)
            sender_ip = addr[0]

            try:
                j = json.loads(data.decode(errors="ignore"))
            except Exception:
                continue

            if j.get("service") != SERVICE:
                continue

            # Trust the sender’s UDP source address (robust across iface changes)
            port = int(j.get("port", 0))
            if not port:
                continue

            key = (sender_ip, port)
            if key == last_key:
                consec[key] += 1
            else:
                last_key = key
                consec[key] = 1

            if consec[key] >= NEED_CONSECUTIVE:
                print(f"{sender_ip}:{port}")
                return 0

        except socket.timeout:
            continue
        except Exception:
            # swallow transient errors and keep listening
            continue
    return 1

if __name__ == "__main__":
    sys.exit(main())
