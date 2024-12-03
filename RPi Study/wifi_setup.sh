#!/bin/bash
read -p "Enter WiFi SSID: " wifi_ssid
read -p "Enter WiFi Password: " wifi_password
echo
if [ -z "$wifi_password" ]; then
    sudo nmcli device wifi connect "$wifi_ssid" ifname wlan0
else
    sudo nmcli device wifi connect "$wifi_ssid" password "$wifi_password" ifname wlan0
fi
if [ $? -eq 0 ]; then
    echo "Successfully connected to $wifi_ssid"
else
    echo "Failed to connect to $wifi_ssid"
fi