# MakersVR
Welcome to MakersVR, an open source VR hardware project focussed around a cheap optical tracking system. <br>
The goal of this project is to create a cheap full body tracking solution and provide a basis to develop custom VR hardware on.

## How does it work?
The basis is an optical tracking systems with two or more cameras observing multiple active markers made with LEDs. <br>
The <a href="https://ar-tracking.com/technology/technical-details/">ART system</a> is quite similar and provides a great overview of the general idea. <br>
Each tracking camera, called <i>Marker Detector</i>, is a Raspberry Pi Zero plus camera. It is executing a Blob Detection algorithm to get the LEDs position in camera space. These are send over a serial connection to a central microcontroller, the <i>Marker Tracker</i>. This serial connection is a CAT6 cable, but is currently used for power, camera sync and UART (might be switched to SPI later). <br>
The centralized <i>Marker Tracker</i> is currently a STMF103 microcontroller, the cheap Blue Pill board. It is connected to up to three <i>Marker Detectors</i> at once as well as the Host PC over a USB connection. It collects the Blob2D data from each <i>Marker Detector</i> and currently immediately sends it to the Host PC using a isochronous Full-Speed USB 2.0 connection. It also times the frame of each <i>Marker Detector</i> down to 0.1ms and sends synchronization feedback back over the serial connection. <br>
On the Host PC, the <i>Configurator</i> program (and later the driver) connects to the USB device and 2D Blobs are streamed in. These 2D Blobs are converted to 3D rays using the position of the cameras in the room and the poses of the markers are inferred. This is still in debate on how exactly this will work, there are several approaches that I will be trying out. <br>
The more detailed references can be found in the Design folder, alongside a preliminary parts list. This places the currently expected costs between 110€ and 200€, depending on how many and which cameras are used.

## Current State
The <i>Marker Detector</i> and <i>Marker Tracker</i> are working, and for the most part all technical challenges for a first prototype are overcome. However, the current blob detection backend of the <i>Marker Detector</i> is suboptimal and will be replaced for the final version. This alternative backend, which operates directly on the QPU of the VideoCore IV, is mostly done, but faces some final challenges which I will focus on after the rest of the system works. <br>
<p align="center">
  <img alt="Hardware Prototype of MakersVR" src="https://github.com/Seneral/MakersVR/raw/master/Design/HW_Prototype_Annotated.jpg" width="50%"/>
  <br>
  The <i>Marker Detector</i> and <i>Marker Tracker</i> prototypes
</p>
The actual tracking system on the host PC is not yet done and will still require a lot of work. This is planned to be developed solely with software-generated data first, so it does not require the above prototype hardware to develop for fast iteration. Currently, I have a physical 4-point marker as seen below, but I am considering adapting a similar marker concept to the <a href="https://ar-tracking.com/products/markers-targets/markers/">ART System</a> with 3D-distances identifying each blob in addition to a fixed calibration pattern. <br>
<p align="center">
  <img alt="Software Configurator of MakersVR" src="https://github.com/Seneral/MakersVR/raw/master/Design/SW_Configurator_Annotated.png" width="60%"/>
  <br>
  The <i>Configurator</i> streaming Blobs from the Prototype Hardware
</p>
One hardware problem I'm still facing is the camera selection. The ecosystem of cameras for the Raspberry Pi is very limited, with the official modules having terribly low field of view, resulting in a smaller tracking volume, and most inofficial modules basing on the older camera module with much worse performance or a completely different chip altogether with questionable support. Currently, I'm seeing if the Official Camera Module V2 with limited Field of View is viable: <br>
<p align="center">
  <img alt="Software Configurator of MakersVR" src="https://github.com/Seneral/MakersVR/raw/master/Design/HW_CameraTrackingVolume.png" width="60%"/>
  <br>
  Tracking volume of three Marker Detectors using the Camera Module V2 in a small room
</p>

## Going Forward
Over the next three months (Exam phase and much free time) I will have a lot of time to develop this further, with at least a working prototype expected at the end of it. <br>
The roadmap: <br>
1. Developing multi-camera tracking algorithm and the marker shape alongside each other. This will be done using software generated data first.
2. Testing stability of algorithm using rendered data, fluid motion and occlusion. If needed, iterate
3. Develop calibration algorithms using single-camera pose estimation.
4. Develop hardware of markers (using PCBs or 3D-Prints) and create prototype.
5. Finishing support of multiple cameras and building prototype with three cameras.
6. Testing with real hardware begins, multiple cameras will be tested out
7. Creating a (3D-printed?) case for the electronics

Along the way, depending on interest, I will expand the documentation on this project

## Sub-Projects
- Marker Detector: Software of the Raspberry Pi
  - <a href="https://github.com/Seneral/VC4CV">VC4CV</a>: The backends (GL and QPU) of the blob detection
- Marker Tracker: Software for the STM32F103 that interfaces with the Marker Detectors (UART /w DMA) and the Host PC (USB)
- Configurator: Test and future Configuration Software of the Host PC. Will be split later into tracking, configurator and driver

## Tracking Quality
All I can deliver about tracking quality right now are calculations, since there is no working version yet. Here are some numbers for the current prototype and the final version: <br>
#### Latency
The current prototype uses the GL backend of VC4CV, and this is the biggest issue right now.
- The camera processing time before reaching my code is unknown, but below 5ms
- For a 1280x720 frame, only 25fps can be reached, which is a latency of about 40ms just for the camera.
- The UART connection is currently only 11520 baud, so another ~8ms is added for just three markers
- STM32 adds little to no latency, and considering the frame time and delay on host side, another 2-4ms can be added

This would add up to around 55ms, just to get it on the PC, so maybe 70-80ms to the display. <br>
For the final version, the QPU backend of VC4CV is used, and here I can only make educated guesses.
- The camera processing time before reaching my code is unknown, but below 5ms
- A 1640x1232 frame at 40fps is targetted (not possible with GL backend), with an estimated frametime of 5-10ms
- While not tested, a baud rate of up to 1MHz (UART or SPI) should be possible, which would take around 1ms
- Again the STM32 and USB protocol add around 2-4ms

This would amount to around 15ms tracking system latency, and 30-40ms motion-to-photon latency. <br>
In the final version, the camera synchronisation measures the exact latency times as a byproduct, so I will be able to name exact numbers when that is done.

#### Accuracy
Here I'm calculating the pixel size at a given distance (<i>dist</i>) with a given horizontal FoV (<i>hfov</i>) and horizontal sensor resolution (<i>hres</i>): <br>
  tan(<i>hfov</i>/2) * 2 * * <i>dist</i> / <i>hres</i> <br>
Here are the numbers for some camera configurations at 3m distance (average center), accounting for binning and partial sensor modes: <br>
1280x720 @ 53.5° hFoV (Camera Module V1): 2.36mm <br> 
1280x720 @ 62.2° hFoV (effectively 48.54°) (Camera Module V2, current prototype with GL backend): 2.21mm <br>
1640x1232 @ 62.2° hFoV (Camera Module V2, later version with QPU backend): 2.21mm <br>
With subpixel blob accuracy and multiple blobs per marker we can hope for 1mm-accuracy, however this remains to be seen.

## Contact
Depending on what you want:
- Create an issue on this repository for involved technical discussions
- Catch me on Discord (Seneral#8110) for quick questions / discussions
- Send me an email otherwise (contact@seneral.dev)

## License
Refer to each individual sub-project (MakersVR_\*) for their licenses. Currently, all are licensed under MPL-2.0 <br>
The rest of the materials in this repository are licensed under the MIT license.

