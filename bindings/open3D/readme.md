# 3D Time of Flight : Open3D bindings

### Instalation
If you build open3D from source, make sure to enable option `GLIBCXX_USE_CXX11_ABI` when running cmake.
For example:
```
cmake -DGLIBCXX_USE_CXX11_ABI=ON
```

To install open3D go to: [Open3D](http://www.open3d.org/docs/release/compilation.html)

When building the aditof project specify the cmake option -DWITH_OPEN3D=ON. 
Add the path to the Open3D installation folder in the cmake variable -DCMAKE_PREFIX_PATH=<path_to_open3d_installation>.

#### Directory Structure

| Directory/File | Description |
| --------- | ----------- |
| showPointCloud | Contains a basic example that displays a pointcloud build with data from aditof|
| CMakeLists.txt | Rules to build the bindings and the examples |
