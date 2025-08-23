#!/usr/bin/env bash
set -euo pipefail

# === configurable knobs (env‑overridable) ===
BASE_IP="${BASE_IP:-192.168.1.50}"     # base station IP
PORT="${PORT:-5004}"                   # RTP/UDP port
W="${W:-640}"                          # width
H="${H:-480}"                          # height
FPS="${FPS:-30}"                       # frame rate
BITRATE="${BITRATE:-4000000}"          # ~4 Mbps
GOP="${GOP:-30}"                       # keyframe interval (frames)
LOG="${LOG:-/var/log/pi_cam_stream.log}"

echo "[pi_cam_stream] -> ${BASE_IP}:${PORT} ${W}x${H}@${FPS} ${BITRATE}bps (GOP=${GOP})"
echo "[pi_cam_stream] starting at $(date)" | tee -a "$LOG"

# rpicam-vid (H.264) -> stdout -> GStreamer payload to RTP/UDP
# Notes:
# --inline: SPS/PPS on every IDR (helps late-joiners)
# --intra: IDR period (frames) ~ your GOP
# --profile baseline: no B-frames for low latency
# --timeout 0: run forever
rpicam-vid \
  --codec h264 \
  --width "${W}" --height "${H}" \
  --framerate "${FPS}" \
  --bitrate "${BITRATE}" \
  --intra "${GOP}" \
  --inline \
  --profile baseline \
  --timeout 0 \
  -o - \
| gst-launch-1.0 -v \
    fdsrc \
    ! h264parse config-interval=1 \
    ! rtph264pay pt=96 \
    ! udpsink host="${BASE_IP}" port="${PORT}" sync=false async=false \
  2>&1 | tee -a "$LOG"
