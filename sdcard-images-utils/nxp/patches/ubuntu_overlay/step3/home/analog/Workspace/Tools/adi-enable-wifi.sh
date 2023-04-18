if [ "$#" -eq 2 ]; then
  wpa_passphrase $1 $2  | sudo tee /etc/wpa_supplicant/wpa_supplicant-wlan0.conf
  sudo systemctl enable wpa_supplicant@wlan0
  sudo reboot
fi

echo "$0 <network SSID> <WiFi password>"
