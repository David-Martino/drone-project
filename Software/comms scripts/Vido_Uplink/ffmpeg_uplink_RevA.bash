#Install this
#sudo apt update
#sudo apt install -y \
  #gstreamer1.0-tools gstreamer1.0-libcamera \
  #gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  #gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  #v4l-utils
  
  #Quick camera sanity check:
  #libcamera-hello -t 2000

#!/usr/bin/env bash
set -euo pipefail

# === configurable knobs ===
BASE_IP="${BASE_IP:-192.168.1.50}"     # base station IP
PORT="${PORT:-5004}"                   # RTP/UDP port
W="${W:-640}"                          # width
H="${H:-480}"                          # height
FPS="${FPS:-30}"                       # frame rate
BITRATE="${BITRATE:-4000000}"          # ~4 Mbps
GOP="${GOP:-30}"                       # keyframe interval (frames)
LOG="${LOG:-/var/log/pi_cam_stream.log}"

#save to usr/local/bin/pi_cam_stream.sh
#chmod +x /usr/local/bin/pi_cam_stream.sh (permissions)

echo "[pi_cam_stream] -> ${BASE_IP}:${PORT} ${W}x${H}@${FPS} ${BITRATE}bps (GOP=${GOP})"
echo "[pi_cam_stream] starting at $(date)" | tee -a "$LOG"

# Low-latency RTP/UDP H.264 pipeline (hardware encode on Pi)
# Notes:
# - libcamerasrc = camera
# - v4l2h264enc uses the Pi's HW encoder
# - config-interval=1 inserts SPS/PPS regularly (resilient to packet loss / late joins)
# - sync=false/async=false keep latency tight
# - apps that join mid-stream (ffplay/VLC) will lock quickly on next keyframe
GST_DEBUG="${GST_DEBUG:-0}" \
gst-launch-1.0 -v \
  libcamerasrc \
    ! video/x-raw,width=${W},height=${H},framerate=${FPS}/1 \
  ! v4l2convert \
  ! v4l2h264enc max-bitrate=${BITRATE} iframe-period=${GOP} \
  ! h264parse config-interval=1 \
  ! rtph264pay pt=96 \
  ! udpsink host="${BASE_IP}" port=${PORT} sync=false async=false \
  2>&1 | tee -a "$LOG"
