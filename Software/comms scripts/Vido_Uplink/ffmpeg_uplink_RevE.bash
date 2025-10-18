#!/usr/bin/env bash
set -euo pipefail

# ---- Config (env-overridable) ----
BEACON_PORT="${BEACON_PORT:-9000}"
DISCOVERY_TIMEOUT="${DISCOVERY_TIMEOUT:-20}"   
SERVICE_NAME="${SERVICE_NAME:-GS-H264}"

BASE_IP="${BASE_IP:-}"                        
PORT="${PORT:-5004}"

W="${W:-640}"; H="${H:-480}"; FPS="${FPS:-30}"
BITRATE="${BITRATE:-4000000}"; GOP="${GOP:-30}"


target_ip=""; target_port=""
if command -v /usr/local/bin/discover_gs.py >/dev/null 2>&1; then
  if out=$(BEACON_PORT="$BEACON_PORT" DISCOVERY_TIMEOUT="$DISCOVERY_TIMEOUT" SERVICE_NAME="$SERVICE_NAME" /usr/local/bin/discover_gs.py 2>/dev/null); then
    target_ip="${out%%:*}"; target_port="${out##*:}"
  fi
fi

# ---- 2) Final fallback: BASE_IP (if provided) ----
if [[ -z "${target_ip}" && -n "${BASE_IP}" ]]; then
  target_ip="$BASE_IP"; target_port="$PORT"
fi

if [[ -z "${target_ip}" ]]; then
  echo "[uplink] ERROR: No ground station found (beacon or BASE_IP)."
  exit 1
fi

echo "[uplink] → ${target_ip}:${target_port}  ${W}x${H}@${FPS}  ${BITRATE}bps (GOP=${GOP})"

# ---- Stream (RTP/H.264) ----
rpicam-vid --codec h264 \
  --width "$W" --height "$H" --framerate "$FPS" \
  --bitrate "$BITRATE" --intra "$GOP" --inline --profile baseline \
  --timeout 0 -o - \
| gst-launch-1.0 -q fdsrc ! h264parse \
    ! rtph264pay pt=96 config-interval=1 \
    ! udpsink host="$target_ip" port="$target_port" sync=false

#NOTEABLE CHANGES: Uses RTP with auto ip detection for GS

#/usr/local/bin/ffmpeg_uplink.sh
#sudo chmod +x /usr/local/bin/ffmpeg_uplink.sh

#For a temp view:
#cat <<'SDP' | ffplay -fflags nobuffer -flags low_delay -probesize 32 -analyzeduration 0 \
 # -protocol_whitelist file,udp,rtp,pipe -i -
#v=0
#o=- 0 0 IN IP4 0.0.0.0
#s=Pi H264
#c=IN IP4 0.0.0.0
#t=0 0
#m=video 5004 RTP/AVP 96
#a=rtpmap:96 H264/90000
#a=fmtp:96 packetization-mode=1
#SDP
