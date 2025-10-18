#!/usr/bin/env python3
# Requires: sudo pip install scapy netifaces
from scapy.all import arping
import ipaddress
import netifaces as ni
import sys

# ---- CONFIG ----
DRONE_MAC = "D0:CF:13:31:1A:E9".lower()   # hardcode your drone's STA MAC here
IFACE     = ""                       # set to your Wi-Fi iface, or ""/None to auto-pick
TIMEOUT_S = 2.0                           # ARP scan timeout
# ---------------

def pick_iface(preferred):
    ifaces = ni.interfaces()
    if preferred:
        if preferred in ifaces:
            return preferred
        print(f"[ERR] Interface '{preferred}' not found.", file=sys.stderr)
        print(f"[INFO] Available: {', '.join(ifaces)}", file=sys.stderr)
        sys.exit(1)
    # auto-pick first non-loopback with IPv4
    for iface in ifaces:
        if iface == "lo":
            continue
        if ni.ifaddresses(iface).get(ni.AF_INET):
            return iface
    print("[ERR] No non-loopback IPv4 interface found.", file=sys.stderr)
    print(f"[INFO] Available: {', '.join(ifaces)}", file=sys.stderr)
    sys.exit(1)

def iface_subnet_cidr(iface: str) -> str:
    addr_list = ni.ifaddresses(iface).get(ni.AF_INET)
    if not addr_list:
        print(f"[ERR] Interface '{iface}' has no IPv4 address.", file=sys.stderr)
        sys.exit(1)
    addr = addr_list[0]
    ip = addr.get("addr"); nm = addr.get("netmask")
    if not ip or not nm:
        print(f"[ERR] Could not obtain IPv4 addr/netmask for '{iface}'.", file=sys.stderr)
        sys.exit(1)
    ipif = ipaddress.IPv4Interface(f"{ip}/{nm}")
    return str(ipif.network)  # e.g. '172.20.10.0/28'

def main():
    iface = pick_iface(IFACE)
    subnet = iface_subnet_cidr(iface)

    try:
        ans, _ = arping(subnet, iface=iface, timeout=TIMEOUT_S, verbose=False)
    except PermissionError:
        print("[ERR] Raw sockets required. Re-run with sudo.", file=sys.stderr)
        sys.exit(2)
    except Exception as e:
        print(f"[ERR] arping failed: {e}", file=sys.stderr)
        sys.exit(1)

    for s, r in ans:
        mac = r.hwsrc.lower()
        ip  = r.psrc
        if mac == DRONE_MAC:
            print(ip)   # print only the IP for easy scripting
            sys.exit(0)

    print("NOT_FOUND", file=sys.stderr)
    sys.exit(1)

if __name__ == "__main__":
    main()
