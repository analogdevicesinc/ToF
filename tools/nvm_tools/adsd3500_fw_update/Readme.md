
1). Steps to follow on windows machine and copy the below files to NXP host platform using Winscp application
    -> Download and Install WinSCP 
    -> Open WinScp 
    -> Host Name (or IP Address) : 10.42.0.1
    -> NXP Username : analog ; Password : analog
    -> Copy the below listed folder from Package to /home/analog/Workspace/
        - Firmware_update_utilities

    -> Copy the directory from Host PC to NXP Host. (Alternate method instead of Winscp) 

	$ pscp -P 22 -l analog -pw analog -r Firmware_update_utilities analog@10.42.0.1:/home/analog/Workspace

2). SSH into NXP platform
    -> SSH with Putty example
    -> Host Name (or IP Address) : 10.42.0.1 ; Port 22 ; SSH
    -> NXP Username : analog
	   Password : analog
	
	$ cd Firmware_update_utilities


3) And then execute the make clean and make commands.
	
	$ make clean
	$ make

4) After execution of above commands the firmware_update executable file will be created and use this file for firmware update, pass the "Fw_Update_X.X.X.bin" file as an argument.

	Note:
	  i) Copy the latest Fw_Update_X.X.X.bin file to Firmware_update_utilities.
	 ii) Below mentioned file, which is passed as an argument to executable file is just an example bin file.
	
	$ sudo ./firmware_update Fw_Update_4.1.7.bin

