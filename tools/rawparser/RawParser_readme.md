* rawparser.py is a python utility to extract Depth, AB, Confidence and XYZ from the frames captured via Data collect or from ADI ToF Viewer.

Command Usage: 
	
	rawparser.py [-h] [--filename FILENAME]

	Script to parse a raw file and extract different frame data

	options:
	  -h, --help           show this help message and exit
	  --filename FILENAME  filename to parse

Example:

	rawparser.py -filename <outfile path>
	
	- The above command will extract Depth, AB, Confidence and XYZ from the outfile and save to outfile_parsed directory.