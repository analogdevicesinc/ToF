Host-boot ADSD3500

ATTENTION! host_boot.py require direct access to ADSD3500 I2C device. Make sure that adsd3500 driver is not loaded and ADSD3500 is visible on i2c bus (i2cdetect).

1.	The host_boot.py script loads the ADSD3500 boot firmware through I2C
        ADSD3500 contain a BootROM bootloader which makes possible this host-boot usecase
2.	Run "python3 host_boot.py" for loading the default "walden_lite.stream" to address 0x38
		Parameters are accepted and other .stream file or I2C address can be specifyed
		"python3 host_boot.py walden_lite.stream 0x38"