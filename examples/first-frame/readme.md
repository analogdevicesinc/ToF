# first-frame sample code

## Overview

The first-frame is a cross platform sample code that shows how to use the aditof sdk to configure a camera so that frames can be captured from it.

For example, the run command could look like:
`./first-frame config/config_adsd3500_adsd3100.json`

The first-frame can talk to a remote ToF camera over the network.

For example if the ip address of the remote camera is `10.43.0.1` the run command could look like:

`./first-frame 10.43.0.1 config/config_adsd3500_adsd3100.json`

In addition please ensure the correct mode is selected. This is done by manually editing the code. In the future this will move to a comamnd line option.

See https://github.com/analogdevicesinc/ToF/blob/e551cab2e470f3743a01a49ef4c22fe663d7715e/examples/first-frame/main.cpp#L127
