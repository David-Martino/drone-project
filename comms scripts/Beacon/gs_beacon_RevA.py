#!/usr/bin/env python3
import json, socket, time, os

PORT = int(os.environ.get("BEACON_PORT", "9000"))
VIDEO_PORT = int(os.environ.get("VIDEO_PORT", "5004"))
SERVICE = os.environ.get("SERVICE_NAME", "GS-H264")

def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("1.1.1.1", 80))  # doesn't actually send traffic
        return s.getsockname()[0]
    finally:
        s.close()

def main():
    ip = get_local_ip()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    payload = {
        "service": SERVICE,
        "ip": ip,
        "port": VIDEO_PORT,
        "proto": "rtp-h264",
        "ts": int(time.time())
    }
    msg = (json.dumps(payload) + "\n").encode()
    print(f"[beacon] broadcasting {payload} → 255.255.255.255:{PORT}", flush=True)
    while True:
        sock.sendto(msg, ("255.255.255.255", PORT))
        time.sleep(2)

if __name__ == "__main__":
    main()


#SAVE THIS FILE ON GS TO BEOADCAST IP 
#/usr/local/bin/gs_beacon.py
#TEST: BEACON_PORT=9000 VIDEO_PORT=5004 SERVICE_NAME=GS-H264 /usr/local/bin/gs_beacon.py
