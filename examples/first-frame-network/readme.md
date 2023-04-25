# first-frame-network sample code

## Overview

The first-frame-network is a cross platform sample code that shows how to use the aditof sdk to talk to a remote ToF camera over the network.

For example if the ip address of the target is `10.42.01` the run command should be:

`./first-frame-network 10.42.0.1 config/config_crosby_old_modes.json`

In addition please ensure the correct mode is selected. This is done by manually editing the code. In the future this will move to a comamnd line option.

See https://github.com/analogdevicesinc/ToF/blob/c1205f76c39220ca92f188b62a28c97e539ed6b5/examples/first-frame-network/main.cpp#L99
