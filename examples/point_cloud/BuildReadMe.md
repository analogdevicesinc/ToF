
```
# Build PointCloud
## Windows
Build using the following commands
```sh
mkdir <ROOTDIR>/build
cd <ROOTDIR>/build
```
Note:The location of this README is referred to as ROOTDIR  


```
cmake -DCMAKE_GENERATOR_PLATFORM=x64 ..
cmake --build ./ --config Release
```

```

## Linux
Build using the following commands
```sh
mkdir <ROOTDIR>/build
cd <ROOTDIR>/build
```

cmake -DCMAKE_BUILD_TYPE=Release ..
make
```
