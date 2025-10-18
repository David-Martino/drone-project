Gotcha—those “every ~2s” hiccups are classic symptoms of keyframe spikes + tiny wireless jitter. Let’s smooth it out with a few quick tweaks. Do them in order; you can stop as soon as it’s smooth.

1) Use sane stream params (smaller keyframes)

On the Pi, run the uplink with a shorter GOP so keyframes aren’t huge bursts:

BASE_IP=172.20.10.6 PORT=5004 W=1280 H=720 FPS=30 BITRATE=4000000 GOP=15 ffmpeg_uplink.sh


GOP=15 = keyframe every 0.5 s at 30fps (much smaller IDR bursts than 2s).

If you still see hiccups, try BITRATE=3000000 or W=960 H=540.

2) Make the receiver more tolerant of jitter

On the laptop, add a jitter buffer and keep the sink unsynced:

gst-launch-1.0 -v udpsrc port=5004 caps="application/x-rtp, media=video, encoding-name=H264, payload=96" ^
  ! rtpjitterbuffer latency=60 drop-on-late=true ^
  ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false


latency=60 (ms) gives a tiny cushion; you can try 40–100.

drop-on-late=true avoids building up delay.

3) Tweak the sender for Wi-Fi friendliness (optional but helpful)

If you’re comfortable editing /usr/local/bin/ffmpeg_uplink.sh, add two tweaks to the GStreamer part:

On rtph264pay add config-interval=1 (repeat SPS/PPS every second).

On udpsink add mtu=1200 to avoid Wi-Fi fragmentation.

Example snippet inside the script (conceptually):

... ! h264parse ! rtph264pay pt=96 config-interval=1 \
! udpsink host=$TARGET port=$PORT mtu=1200


And in the rpicam-vid args, make sure you’re using no-preview and inline headers:

Add -n (you already did) and consider --inline (puts SPS/PPS at keyframes).

If the script accepts your GOP, it will map to --intra <GOP> under the hood; if not, add --intra 15 explicitly beside other rpicam flags.

4) Disable Wi-Fi power saving on the Pi

Power-save can create rhythmic stalls. Turn it off and make it persistent:

Immediate (until reboot):

sudo iw dev wlan0 set power_save off


Persist with NetworkManager (replace with your actual connection name):

nmcli con show            # find the name e.g. "iPhone98"
sudo nmcli con modify "iPhone98" 802-11-wireless.powersave 2
sudo systemctl restart NetworkManager

5) Keep the iPhone hotspot “friendly”

Keep Maximize Compatibility ON (2.4 GHz).

Keep phone close to the Pi; avoid walls/metal.

Ensure your laptop is only on the hotspot (no home Wi-Fi IPs showing).

Quick presets to try (in increasing quality)

Rock-solid baseline:
W=640 H=480 FPS=30 BITRATE=2000000 GOP=15

Better, still robust:
W=960 H=540 FPS=30 BITRATE=3000000 GOP=15

720p standard:
W=1280 H=720 FPS=30 BITRATE=4000000 GOP=15

If you want, paste the rpicam-vid/GStreamer lines from your ffmpeg_uplink.sh and I’ll mark exactly where to drop config-interval=1, mtu=1200, and (if needed) --intra 15 --inline.