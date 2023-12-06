### BUILD ADSD3500 DEVICE DRIVER ###

This README file provides instructions on building the ADSD3500 Device driver for the NXP device.

1. Connect the NXP Eval kit to an active internet connection.
2. Install the dependencies for building the device driver by running the following commands in the NXP's terminal.
	$ sudo apt-get install flex
	$ sudo apt-get install bison
3. Generate necessary executables needed for building the driver.
	$ cd /usr/src/linux-imx/
	$ sudo make clean && sudo make prepare
4. Navigate to the current directory and run the make command.
	$ cd ~/Workspace/ToF/drivers/adsd3500/nxp/
	$ make
