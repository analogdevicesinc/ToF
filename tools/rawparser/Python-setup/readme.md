# Welcome to aditofpython bindings

*To setup the python environment run below command:
	.\aditofpython_env.bat

* To activate virtual environment run below command:
	.\activate.bat

* To deactivate virtual environment run below command:
	.\deactivate.bat


Note: Make sure that python3.10 is the default python.

* How to run Python examples?

	-There are 4 python examples:

	   1. First_frame.py:
		
	      This will capture one frame in requested mode and display depth image.
		
		first_frame.py usage:
		USB: first_frame.py <mode name> <config>
		Network connection: first_frame.py <mode name> <ip> <config>

		* Mode names:
		lr-native: 1024 x 1024
		lr-qnative: 512 x 512
		sr-native: 1024 x 1024
		sr-qnative: 512 x 512

		* For example:
		python first_frame.py lr-qnative 10.43.0.1 config\config_adsd3500_adsd3100.json

	   2. saveCCBToFile.py:
	      
              This is used to save sensor calibration file to the host machine 

	         saveCCBToFile.py usage:
		   save_ccb.py <ip> <config>

		* For example:
		python saveCCBToFile.py 10.43.0.1 config\config_adsd3500_adsd3100.json

	   3. skeletal_tracking.py:
	       
 	      This example is used to track a person's skeleton.

	      	skeletal_tracking.py Usage:		
		  python skeletal_tracking.py

	   4. depth-image-animation-pygame.py:
	       
 	      This example is used to live stream depth images from camera.

	      depth-image-animation-pygame.py Usage:
		
		python depth-image-animation-pygame.py


Note: After activating the virtual environment all the examples should be run from <Installed Location>\bin.