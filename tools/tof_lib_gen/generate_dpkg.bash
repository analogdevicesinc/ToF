# Generating dpkg for Crosby
echo "Generating dpkg for Ubuntu 20.04 Amd64"
sudo DOCKER_BUILDKIT=1 docker build --file ubuntu_20_04/crosby/Dockerfile --output crosby/out_ubuntu20 .
echo "Generating dpkg for Ubuntu 22.04 Amd64"
sudo DOCKER_BUILDKIT=1 docker build --file ubuntu_22_04/crosby/Dockerfile --output crosby/out_ubuntu22 .

# enabling read and write 
sudo chmod -R a+rx crosby

# Generating dpkg for adsd3030 (adsd3030)
echo "Generating dpkg for Ubuntu 20.04 Amd64"
sudo DOCKER_BUILDKIT=1 docker build --file ubuntu_20_04/adsd3030/Dockerfile --output adsd3030/out_ubuntu20 .
echo "Generating dpkg for Ubuntu 22.04 Amd64"
sudo DOCKER_BUILDKIT=1 docker build --file ubuntu_22_04/adsd3030/Dockerfile --output adsd3030/out_ubuntu22 .

sudo chmod -R a+rx adsd3030
