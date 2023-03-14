echo "Generating dpkg for Ubuntu 18.04 Amd64"
sudo DOCKER_BUILDKIT=1 docker build --file ubuntu_18_04/Dockerfile --output out_ubuntu18 .
echo "Generating dpkg for Ubuntu 20.04 Amd64"
sudo DOCKER_BUILDKIT=1 docker build --file ubuntu_20_04/Dockerfile --output out_ubuntu20 .
echo "Generating dpkg for Ubuntu 22.04 Amd64"
sudo DOCKER_BUILDKIT=1 docker build --file ubuntu_22_04/Dockerfile --output out_ubuntu22 .
