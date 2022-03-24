# Imshow jetson example

### Overview
This example demonstrates how to capture data from the TOF system on the Nvidia jetson and display it using OpenCV

![Display Image](https://github.com/analogdevicesinc/ToF/blob/master/doc/img/imshow.png)

The following code snippet is used to convert the `aditof::Frame` into a `cv::Mat` containing the depth information
```cpp
/* Convert from frame to depth mat */
cv::Mat mat;
status = fromFrameToDepthMat(frame, mat);
if (status != Status::OK) {
        LOG(ERROR) << "Could not convert from frame to mat!";
        return 0;
}
```
The IR information can be stored in a Mat by using the `fromFrameToIrMat` function instead of `fromFrameToDepthMat`

Only one video stream is available at a time, depth or ir. To enable one or the other set the frame type of the system
```cpp
status = camera->setFrameType("depth_only");
```
or
```cpp
status = camera->setFrameType("ir_only");
```
