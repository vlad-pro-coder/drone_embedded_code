#!/bin/bash

# Disconnect wlan0 from any network
nmcli device disconnect wlan0
# Wait a second
sleep 1
# Bring up the hotspot
sudo nmcli device wifi hotspot ifname wlan0 ssid PiHotSpot password nimeni123

sudo ip route add default via 192.168.1.1 dev wlan0 metric 500
# Optional: print status
nmcli device status
