Scripts to plot what is sent by the camera in TP4 or the FFT in TP5 instead of using Matlab.
The bluetooth should work as well as the USB, except for the connection time 
(but depends on the bluetooth dongle/chip used by the computer).


You have at this time only one possibility to use the scripts :

1) By installing Python3 and running the scripts
 	-Install python3 (Check "pip" and "Add Python to environment variables" if asked during the installation process)
   
	-(for Windows) Type « pip install pyserial» and « pip install matplotlib » in a command line window
	-(for Mac) Type « pip3 install pyserial» and « pip3 install matplotlib » in a terminal window

Usage:

  For TP4:
	-Run the script plotImage.py in the terminal and give to it the com port as argument
	 example : « python path_to_the_script\plotImage.py comX » (for Windows)
	 or      : « python3 path_to_the_script/plotImage.py /dev/cu.usbmodemXXXX » (for Mac)

  For TP5:
	-Run the script plotFFT.py in the terminal and give to it the com port as argument
	 example : « python path_to_the_script\plotFFT.py comX » (for Windows)
	 or      : « python3 path_to_the_script/plotFFT.py /dev/cu.usbmodemXXXX » (for Mac)



Troubleshooting :
If you get a lot of Timeout messages in the terminal while trying to receive something from the e-puck2, quit the program, unplug the USB cable (or disable the bluetooth if used instead of the USB), shutdown the epuck2 and retry.