# DNN Example

### Overview
This example  demonstrates object detection on a combination between the depth frame and the IR frame using the SSD Mobilenet object detection example from OpenCV and the Aditof SDK. It also shows how to compute and display the IR frame.

It works with model taken from [MobileNet-SSD](https://github.com/djmv/MobilNet_SSD_opencv). 
Building the project with CMake will download prototxt and caffemodel, used for object detection. 

For running the python program use:
```console
python dnn.py --prototxt \pathTo\MobileNetSSD_deploy.prototxt  --weights \pathTo\MobileNetSSD_deploy.caffemodel
```

![Display Image](https://github.com/analogdevicesinc/ToF/blob/master/doc/img/dnn_python.PNG) 
