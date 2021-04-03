# MarkerDetector

This is the Camera Blob Detector program running on a Raspberry Pi (Zero) and connecting over a serial connection to the MarkerTracker

## Setup

During development no image is supplied, and setup is based on the Raspian Lite operating system. </br>
In the future I plan to base it upon tinyCore or some other bare OS and provide a small < 50MB image </br>

1. Flash a new Raspbian image onto a SD card </br>
2. Set up wifi and SSH on the boot partition if you will be using SSH (recommended on Zero W) </br>
3. Log in using SSH or keyboard+monitor </br>
4. Enable camera using raspi-config </br>
5. Install build tools: </br>
	`sudo apt install git` </br>
	`sudo apt install cmake` </br>
6. Download source: </br>
	`git glone https://github.com/Seneral/MakersVR` </br>
	`cd MakersVR/MakersVR_MarkerDetector` </br>
7. Build source: </br>
	`mkdir build` </br>
	`cd build` </br>
	`cmake ..` </br>
	`make` </br>
8. Install as service: </br>
	`cd ..` </br>
	`sudo chmod +x *.sh` </br>
	`sudo ./install.sh` </br>
  Then either `./start.sh` or restart the pi

## Hardware Connection

The serial connection is done over a CAT 5e (or CAT 6) cable. Currently, three wires are reserved for ground, two for VCC (5V), two for RX/TX and one for Sync (not yet used). I chose the 6 consecutive ports 2-4-6-8-10-12 on the Raspberry Pi headers, where 2 and 4 are VCC, 6 are the ground wires boundled into one, 8 TX, 10 RX, and 12 probably the one to map the Camera sync output to (tbd). </br>
The only other connection required is the Camera MIPI CSI.