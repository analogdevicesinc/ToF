sudo systemctl stop gunicorn
sudo timedatectl set-ntp no
sudo timedatectl set-time '2015-11-23 08:10:40'
rm /tmp/error.log
rm /tmp/access.log
sudo systemctl daemon-reload
sudo systemctl restart gunicorn
#sleep 5
#sudo systemctl status gunicorn
