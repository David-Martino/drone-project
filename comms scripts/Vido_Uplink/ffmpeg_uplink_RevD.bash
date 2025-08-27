#!/usr/bin/env bash
set -euo pipefail

BASE_IP="${BASE_IP:-172.20.10.4}"
PORT="${PORT:-5004}"
W="${W:-640}"
H="${H:-480}"
FPS="${FPS:-30}"
BITRATE="${BITRATE:-4000000}"
GOP="${GOP:-30}"

rpicam-vid --codec h264 --width "$W" --height "$H" --framerate "$FPS" \
  --bitrate "$BITRATE" --intra "$GOP" --inline --profile baseline \
  --timeout 0 -o - | \
gst-launch-1.0 -v fdsrc ! h264parse config-interval=-1 \
  ! rtph264pay pt=96 config-interval=1 \
  ! udpsink host="$BASE_IP" port="$PORT" sync=false


#/usr/local/bin/ffmpeg_uplink.sh
#sudo chmod +x /usr/local/bin/ffmpeg_uplink.sh

#For a temp view:
cat <<'SDP' | ffplay -fflags nobuffer -flags low_delay -probesize 32 -analyzeduration 0 \
  -protocol_whitelist file,udp,rtp,pipe -i -
v=0
o=- 0 0 IN IP4 0.0.0.0
s=Pi H264
c=IN IP4 0.0.0.0
t=0 0
m=video 5004 RTP/AVP 96
a=rtpmap:96 H264/90000
a=fmtp:96 packetization-mode=1
SDP
