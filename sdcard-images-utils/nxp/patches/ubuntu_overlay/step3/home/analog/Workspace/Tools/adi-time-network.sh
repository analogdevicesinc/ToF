if nc -zw1 google.com 443; then
  echo "Network avaialble, setting up network time sync."
  sudo timedatectl set-ntp yes
else
  echo "Network not aviailable."
fi
