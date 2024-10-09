This README file provides instructions on building the NXP-ADSD3500 driver module and loading the module on the NXP device.
	
### Building ADSD3500 Driver Module on NXP ###

1. Generate necessary executables needed for building the driver module.
	$ cd /usr/src/linux-imx/
	$ sudo make clean && sudo make prepare
2. Navigate to the current directory and run the make command.
	$ cd ~/Workspace/ToF/drivers/adsd3500/nxp/
	$ make

Note: The tools 'flex' and 'bison' are required to build the driver module and they are pre-installed on the NXP SD Card image.

### Loading ADSD3500 Driver module on NXP ###

1. Once the driver module is built, the module can be found in the "out/" folder.
2. Copy the adsd3500.ko file and paste it on the location "/lib/modules/5.10.72-<kernel-version_number>/kernel/drivers/media/i2c/"
3. Restart the NXP.
4. Now the NXP will be booted with the newly generated ADSD3500 kernel module.


