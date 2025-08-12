# Network scan for ip

sudo nmap your_ip/your_subnet

# test video steaming  pi -> groundstation 

ffmpeg -re -f lavfi -i testsrc=size=640x480:rate=30 -c:v libx264 -f mpegts udp://<GCS_IP>:5000

# on ground station

ffplay udp://@:5000

ffplay -fflags nobuffer -flags low_delay -i udp://0.0.0.0:5004

gst-launch-1.0 -v udpsrc port=5004 \
  caps="application/x-rtp, media=video, encoding-name=H264, payload=96" \
  ! rtph264depay ! h264parse ! avdec_h264 \
  ! videoconvert ! autovideosink sync=false


# Basic iperf3 stress test on GS run first

iperf3 -s

# Then on the pi run

iperf3 -c <GCS_IP> -u -b 10M
