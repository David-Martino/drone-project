#!/usr/bin/env bash
# Pi camera → FFmpeg → MPEG-TS/UDP uplink (auto-discovers base station via mDNS)
set -euo pipefail

# -------- base host/IP discovery (mDNS first, with fallbacks) --------
# You can override any of these via systemd Environment=… or exporting before running.
BASE_HOST="${BASE_HOST:-groundstation.local}"   # advertised hostname on the base station
BASE_MAC="${BASE_MAC:-}"                        # optional fallback MAC: AA:BB:CC:DD:EE:FF
RESOLVE_TRIES="${RESOLVE_TRIES:-10}"            # seconds to keep trying while Wi-Fi comes up

resolve_host() { getent hosts "$1" | awk '{print $1}' | head -n1; }
resolve_mdns() { command -v avahi-resolve-host-name >/dev/null 2>&1 && \
                  avahi-resolve-host-name -4 "$1" 2>/dev/null | awk '{print $2}' | head -n1 || true; }

discover_ip() {
  local ip=""
  # 1) explicit override wins
  [[ -n "${BASE_IP:-}" ]] && { echo "$BASE_IP"; return; }
  # 2) NSS (DNS/mDNS via libnss-mdns)
  ip="$(resolve_host "$BASE_HOST")"
  [[ -n "$ip" ]] && { echo "$ip"; return; }
  # 3) direct Avahi query
  ip="$(resolve_mdns "$BASE_HOST")"
  [[ -n "$ip" ]] && { echo "$ip"; return; }
  # 4) optional ARP fallback by MAC (best effort)
  if [[ -n "$BASE_MAC" ]]; then
    ip="$(ip neigh | awk -v want="$(echo "$BASE_MAC" | tr a-z A-Z)" 'toupper($3)==want {print $1; exit}')"
    [[ -n "$ip" ]] && { echo "$ip"; return; }
  fi
  echo ""
}

i=0; BASE_IP="$(discover_ip)"
while [[ -z "$BASE_IP" && $i -lt $RESOLVE_TRIES ]]; do
  sleep 1; i=$((i+1)); BASE_IP="$(discover_ip)"
done

if [[ -z "$BASE_IP" ]]; then
  echo "[uplink] ERROR: cannot resolve '${BASE_HOST}'. Set BASE_IP=… or fix mDNS."; exit 1
fi

echo "[uplink] using base station ${BASE_HOST} → ${BASE_IP}"
# --------------------------------------------------------------------

# -------- stream knobs (can be overridden via env) -------------------
PORT="${PORT:-5004}"
W="${W:-640}"
H="${H:-480}"
FPS="${FPS:-30}"
BITRATE="${BITRATE:-4000000}"   # bits per second
GOP="${GOP:-30}"                # keyframe interval in frames
# --------------------------------------------------------------------

# Sanity checks
command -v rpicam-vid >/dev/null || { echo "[uplink] rpicam-vid not found"; exit 1; }
command -v ffmpeg     >/dev/null || { echo "[uplink] ffmpeg not found";     exit 1; }

# Clean up children on exit
pids=()
cleanup() { for p in "${pids[@]:-}"; do kill "$p" 2>/dev/null || true; done; }
trap cleanup INT TERM EXIT

echo "[uplink] → udp://${BASE_IP}:${PORT}  ${W}x${H}@${FPS}  bitrate=${BITRATE}  GOP=${GOP}"

# rpicam-vid (HW H.264) → FFmpeg (copy) → MPEG-TS over UDP (easy to preview & robust)
# Note: --inline + frequent IDR (GOP) helps late join & packet loss recovery.
rpicam-vid --codec h264 \
  --width "${W}" --height "${H}" --framerate "${FPS}" \
  --bitrate "${BITRATE}" --intra "${GOP}" --inline --profile baseline \
  --timeout 0 -o - \
| ffmpeg -loglevel warning -re -fflags +genpts -i - -c copy -f mpegts \
    "udp://${BASE_IP}:${PORT}?pkt_size=1316&fifo_size=1000000&overrun_nonfatal=1" &
pids+=($!)

# Wait on the pipeline
wait "${pids[0]}"
