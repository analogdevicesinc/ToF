# Useful scripts
* Building cross platform docker images with limited memory usage:
```console
sudo docker buildx build -m 2g --output=type=docker --memory-swap
2g -t <tag_name> --platform linux/arm64/v8 .
```

* Run interactive mode for cross platform docker images
```console
sudo docker run -v /bin/qemu-aarch64-static:/usr/bin/qemu-aarch64-
static --platform linux/arm64/v8 -it <docker_image_name> bash
```

* Tag images 
```console
docker image tag <old_name:latest> <new_name:latest>
```

