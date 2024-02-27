Host-boot ADSD3500

1. The host_boot.sh script loads the ADSD3500 boot firmware through the I2C driver.

2. Execute the below shell script to host boot the connected device.
        $ sudo ./host_boot.sh

3. Change the directory to NVM_Utils and follow the steps mentioned in the readme.txt file.
        $ cd NVM_Utils

4. Execute the below script to self boot the connected device.

        $ cd ..

        $ sudo ./self_boot.sh

Note: Host boot scripts will boot the ADSD3500 from the connected host. It is recommended to do host boot only when ADSD3500 fails to self-boot.
