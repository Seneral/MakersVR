# MarkerTracker

This is the simple software running on the STM32F103, aka BluePill, that is the middleman between the cameras and the host. It serves as an interface for the serial interface over the CAT cables, synchronizes the cameras (tbd) and funnels all camera blob outputs through one USB 2.0 connection.

## Setup

This is mainly general instructions for developing for the STM32F103, aka BluePill, for which there are a ton of separate tutorials.

### Hardware Requirements
- STM32F103 development board. Most likely the Blue Pill (around 2.5€), with headers soldered on
- A programmer. I use a simple FTDI programmer with 3.3V output (IMPORTANT!), but you could use on of those ST-Links as well if you want debugging support

### Software Requirements
1. Get the <a href="https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm">GNU Arm Embedded Toolchain</a>, and make sure it's in the path </br>
	(I used 9-2019-q4-major, anything after that should work just fine)
2. Get the <a href="https://www.st.com/en/development-tools/stm32cubeprog.html">STM32CubeProgrammer</a>, and make sure it's in the path
3. Now you could either use VSCode (with everything set up already), or adapt what you can find in `.vscode/tasks.json`. These are simple commands and should be self-explanatory.

### Hardware Setup
- Make sure you have a Pinout of the STM32F103 handy
- Connect your programmer to TX1/RX1 ports on PA9 and PA10, as always RX-TX switched
- Then connect the 3.3V (!!) and GND pins
- Then make sure the Yellow Jumpers are in the programming position (look it up)
- Then plug your USB programmer in (Not the board itself)

### Software Setup
- Test if you can build the project (in VS Code Ctrl-Shift-P, Run Task, Build)
- Windows:
	- Figure out the COM port of your USB programmer and replace it in .vscode/tasks.json in port=COMx
- Linux:
	- Figure out the device path of your USB programmer (look at dmesg) and replace it in .vscode/tasks.json in port=/dev/...
	- To make the USB programmer accessible, either:
		- Make sure you are in the `dialout` group, if not: `sudo adduser $USER dialout` and then restart
		- Execute STM32_Programmer_CLI with sudo (not possible with VS Code tasks)
- Test if you can upload the build (in VS Code Ctrl-Shift-P, Run Task, Upload)

Once uploaded, you can change the Yellow Jumpers back to the working position, unplug the programmer and plug in your STM32F103. It should show up as "MakersVR Device" in the Windows Device Manager and should be able to connect in the Configurator. </br>
If you are on Linux, you might face a permission error in the Configurator program, in that case see the Configurator README for instructions how to allow access to the USB device without using sudo.
## Hardware Connection

The serial connection to each camera is done over a CAT 5e (or CAT 6) wire. Currently, three wires are reserved for ground, two for VCC (5V), two for RX/TX and one for Sync (not yet used). </br>
On the Microcontroller side, you'll want to power all cameras through a separate USB power supply as three cameras will likely draw more than 500mA. </br>
To do this, use a USB breakout board to connect to all three ground pins of all three RJ45 ports, and make sure to also connect it to the ground of the STM32F103 (so they share a ground). The VCC output can be directly connected to the two VCC pins on each RJ45 port. </br>
As for the RX/TX pins, they are connected to the three UART ports on the STM32F103 Blue Pill - note that the UART 1 port has two sets of pins, one is used for programming (A9/A10), and one for the camera (B6/B7). The other two are A2/A3 and B10/B11. Make sure you connect TX from the Raspberry to the RX of the Microcontroller and vice versa. </br>
The sync pins are not yet used and no port has been finally choosen either. </br>
Also make sure you know exactly which pins on the RJ45 port correspond to which pins on the other side - you should have a straight cable (without cross-over) so it should be pretty straight forward. </br>
The way I chose to occupy my wires is as follows according to <a href="https://www.warehousecables.com/pictures/W1siZiIsIjIwMTgvMDkvMjUvODZtaDJ3dGF3Ml9yajQ1X3Q1NjhhLnBuZyJdXQ/rj45-t568a.png?sha=d8cbafbf54b968b9">this wire order</a>: </br>
VCC - VCC - TX - RX - GND - GND - GND - SYNC </br>
This should pair TX/RX each with a ground, pair VCC together and pair the sync with the last ground.