if [ "$#" -eq 2 ]; then
  ssid="$1"
  password="$2"
else
  # Fall back to reading SSID/password from stdin (one per line) so callers
  # don't need to pass credentials as command-line arguments.
  read -r ssid
  read -r password
fi

if [ -n "$ssid" ] && [ -n "$password" ]; then
  wpa_passphrase "$ssid" "$password" | sudo tee /etc/wpa_supplicant/wpa_supplicant-wlan0.conf
  sudo systemctl enable wpa_supplicant@wlan0
  sudo reboot
else
  echo "$0 <network SSID> <WiFi password>"
fi
