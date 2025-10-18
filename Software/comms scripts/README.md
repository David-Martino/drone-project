# Erwin's Attempt at Implementing David's Scripts

1. Install `Raspberry Pi Imager` on Windows (Ubuntu dual boot wasn't seeing the SD card).
1. Plug in micro SD card (in standard SD card adapter) with write lock off (if present).
1. Choose Device as `Raspberry Pi Zero 2 W`.
1. Choose OS as `Raspberry Pi OS Lite (64-bit)` under Raspberry Pi OS (Other).
1. Choose Storage as the micro SD card you plugged in.
1. Click Next and configure OS settings:
    1. Set username and password (`erwin`, `12345678`).
    1. Configure wireless LAN (`iPhone98`, `12345678`, `AU`).
    1. Set locale settings (`Australia/Perth`, `au`).
    1. Enable SSH (select `Use password authentification`).
1. Continue to flash the SD card.
1. On iPhone hotspot, turn on `Allow Others to Join` and `Maximise Compatibility` to force 2.4GHz wifi.
1. Plug micro SD card into Pi Zero 2 W and start up, with keyboard and monitor connected.
1. Once booted, log in with username and password. 
1. Wait a minute and check if connected to the hotspot with `nmcli device status` and `hostname -I`.
1. If not connected, configure wifi settings with:
    1. Confirm hotspot is visible with `nmcli dev wifi list`.
    1. Connect to hotspot with `sudo nmcli dev wifi connect "iPhone98" password "12345678" ifname wlan0`.
    1. Check connection is in list of saved connections with `nmcli con show`.
    1. Configure autoconnection with `sudo nmcli con mod "iPhone98" connection.autoconnect yes`.
    1. Reboot with `sudo reboot`.
    1. Wait a minute and check if connected to the hotspot with `nmcli device status` and `hostname -I`.
1. Connect to the hotspot with your groundstation.
1. Test SSH connection from your groundstation with `ssh erwin@raspberrypi.local` and `12345678`. 
1. Open a terminal on the groundstation, navigate to `drone-project\comms script\Beacon\`, and run `python .\gs_beacon.py`.
1. Install GStreamer from `https://gstreamer.freedesktop.org/download/#windows` (download and run `MSVC 64-bit runtime installer`).
1. Add GStreamer to path (`C:\Program Files\gstreamer\1.0\msvc_x86_64\bin` on Windows).
1. Launch GStreamer with `gst-launch-1.0 -v udpsrc port=5004 caps="application/x-rtp, media=video, encoding-name=H264, payload=96" ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false`. 
1. On the Pi (or via SSH), install with the following:
    ```
    sudo apt update
    sudo apt full-upgrade -y
    sudo reboot
    sudo apt -y install gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly ffmpeg python3-pip rpicam-apps iperf3
    ```
1. On the groundstation, navigate to `drone-project\comms script\` and copy with the following:
    ```
    scp Beacon\discover_gs.py Startup\Pi-Startup-FFmpeg.service_RevC.service Vido_Uplink\ffmpeg_uplink_RevE.bash erwin@raspberrypi.local:/home/erwin/
    ```
1. On the Pi (or via SSH), move and rename files with the following:
    ```
    sudo mv ~/discover_gs.py /usr/local/bin/
    sudo mv ~/ffmpeg_uplink_RevE.bash /usr/local/bin/ffmpeg_uplink.sh
    sudo chmod +x /usr/local/bin/discover_gs.py /usr/local/bin/ffmpeg_uplink.sh
    sudo mv ~/Pi-Startup-FFmpeg.service_RevC.service /etc/systemd/system/pi-uplink.service
    sudo systemctl daemon-reload
    ```
1. On the Pi (or via SSH), use `sudo nano /etc/systemd/system/pi-uplink.service` to edit the User from `piteam05` to `root`, then save (CTRL+O, ENTER) and exit (CTRL+X).
1. Fix Windows line endings with:
    ```
    sudo apt -y install dos2unix
    sudo dos2unix /usr/local/bin/ffmpeg_uplink.sh /usr/local/bin/discover_gs.py /etc/systemd/system/pi-uplink.service
    ```
1. On the Pi (or via SSH), disable the Pi video preview with `sudo sed -i '0,/rpicam-vid /s//rpicam-vid -n /' /usr/local/bin/ffmpeg_uplink.sh`.  
1. On the Pi (or via SSH), set environment variables and launch streaming with `BASE_IP=172.20.10.6 PORT=5004 W=1280 H=720 FPS=30 BITRATE=4000000 GOP=15 ffmpeg_uplink.sh`. End with `CTRL+C`.
1. On the Pi (or via SSH), change the uplink service details with `sudo nano /etc/systemd/system/pi-uplink.service` to match the following:
    ```
    [Unit]
    Description=Pi Camera → RTP/UDP uplink (beacon + IP fallback)
    After=network-online.target
    Wants=network-online.target

    [Service]
    Type=simple
    User=root
    Group=video
    # Discovery config:
    Environment=BEACON_PORT=9000
    Environment=DISCOVERY_TIMEOUT=20
    Environment=SERVICE_NAME=GS-H264
    # Final fallback:
    Environment=BASE_IP=172.20.10.6
    # Video params:
    Environment=PORT=5004
    Environment=W=1280
    Environment=H=720
    Environment=FPS=30
    Environment=BITRATE=4000000
    Environment=GOP=15
    ExecStartPre=/bin/sleep 3
    ExecStart=/usr/local/bin/ffmpeg_uplink.sh
    Restart=always
    RestartSec=2

    [Install]
    WantedBy=multi-user.target
    ```
1. On the Pi (or via SSH), test the service with:
    ```
    sudo systemctl daemon-reload
    sudo systemctl start pi-uplink.service
    journalctl -u pi-uplink.service -n 20 --no-pager
    ```
1. On the Pi (or via SSH), stop the service and enable autostart with:
    ```
    sudo systemctl stop pi-uplink.service
    sudo systemctl enable --now pi-uplink.service
    ```
1. On the Linux groundstation, install with:
    ```
    sudo apt update && sudo apt -y install \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    ffmpeg \
    python3 \
    python3-pip \
    iperf3
    ```
1. On the Linux groundstation, mnually test receiving the video stream with:
    ```
    gst-launch-1.0 -v udpsrc port=5004 \
    caps="application/x-rtp, media=video, encoding-name=H264, payload=96" \
    ! rtpjitterbuffer latency=60 \
    ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false
    ```


To start streaming:
1. Turn on hotspot with `Maximise Compatibility` on.
1. Connect to hotspot from groundstation.
1. CD to `drone-project` directory in terminal and run `make stream`.
1. Power up the Raspberry Pi Zero 2 W, and streaming should start within 30 seconds.

Run `make stop` to end streaming and beaconing on the groundstation.