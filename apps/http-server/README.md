## HTTP-Network server

With the help of this http-server the user is able to connect to camera via http protocol, to configure and request frames from the Time of Flight module using the SDK on target.

Limitations:
- port 7681 is reserved for this communication


## Structure overview

![Structure overview](doc/img/http-server.png)


## Start server

To build the SDK together with http-server:

```console
cmake -DNXP=1 -DWITH_HTTP_SERVER=1 -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" ..
make
```
To start the server on the target run the following command:
```console
./build/apps/http-server/aditof-http-server
```


## UI interface

* Enter IP address:
![IP address](doc/img/http-server-step-0.png)

* Select camera:
![Select camera](doc/img/http-server-step-1.png)

* Select frame type:
![Frame type](doc/img/http-server-step-2.png)

* Select frame format:
![Frame format](doc/img/http-server-step-3.png)

* Start/Stop state:
![Ready state](doc/img/http-server-step-4.png)

* Streaming:
![Streaming](doc/img/http-server-step-5.png)