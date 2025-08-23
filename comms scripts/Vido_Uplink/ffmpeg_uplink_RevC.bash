#/usr/local/bin/ffmpeg_uplink.sh
set -euo pipefail

BASE_IP="${BASE_IP:-192.168.1.50}"
PORT="${PORT:-5004}"
W="${W:-1280}"
H="${H:-720}"
FPS="${FPS:-30}"
BITRATE="${BITRATE:-4000000}"
GOP="${GOP:-30}"

# rpicam-vid → FFmpeg → MPEG-TS over UDP (lowest hassle)
exec rpicam-vid --codec h264 \
  --width "$W" --height "$H" --framerate "$FPS" \
  --bitrate "$BITRATE" --intra "$GOP" --inline --profile baseline \
  --timeout 0 -o - \
| ffmpeg -loglevel warning -re -fflags +genpts -i - -c copy -f mpegts \
  "udp://$BASE_IP:$PORT?pkt_size=1316&fifo_size=1000000&overrun_nonfatal=1"


#sudo chmod +x /usr/local/bin/ffmpeg_uplink.sh
