# Network scan for ip

sudo nmap your_ip/your_subnet

# test video steaming  pi -> groundstation 

ffmpeg -re -f lavfi -i testsrc=size=640x480:rate=30 -c:v libx264 -f mpegts udp://<GCS_IP>:5000

# on ground station

ffplay udp://@:5000

# Basic iperf3 stress test on GS run first

iperf3 -s

# Then on the pi run

iperf3 -c <GCS_IP> -u -b 10M
