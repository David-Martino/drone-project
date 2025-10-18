# Location of Wifi file
sudo nano /etc/wpa_supplicant/wpa_supplicant.conf


# Restarting Wifi
sudo wpa_cli reconfigure

# Field structure Example

network={
    ssid="UWA"                    # Wi-Fi network name
    key_mgmt=WPA-EAP              # Enterprise WPA2 authentication type
    eap=PEAP                      # Outer EAP method
    identity="student@uwa.edu.au" # Your login username
    password="yourpassword"       # Your Wi-Fi password
    phase1="peaplabel=0"          # TLS tunnel setting for PEAP
    phase2="auth=MSCHAPV2"        # Inner authentication method
    priority=10                   # Connection priority
}
