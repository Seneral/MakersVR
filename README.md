# MakersVR
Welcome to MakersVR, an open source VR hardware project focused around a cheap optical tracking system. <br>
The goal of this project is to create a cheap full body tracking solution and provide a basis to develop custom VR hardware on.

## How does it work?
The basis is an optical tracking systems with two or more cameras observing multiple active markers made with LEDs. <br>
The <a href="https://ar-tracking.com/technology/technical-details/">ART system</a> is quite similar and provides a great overview of the general idea. <br>
Each tracking camera, called <i>Marker Detector</i>, is a Raspberry Pi Zero plus camera. It is executing a Blob Detection algorithm to get the LEDs position in camera space. These are send over a serial connection to a central microcontroller, the <i>Marker Tracker</i>. This serial connection is a CAT6 cable, but is currently used for power, camera sync and UART (might be switched to SPI later). <br>
The centralized <i>Marker Tracker</i> is currently a STM32F103 microcontroller, the cheap Blue Pill board. It is connected to up to three <i>Marker Detectors</i> at once as well as the Host PC over a USB connection. It collects the Blob2D data from each <i>Marker Detector</i> and currently immediately sends it to the Host PC using a isochronous Full-Speed USB 2.0 connection. It also times the frame of each <i>Marker Detector</i> down to 0.1ms and sends synchronization feedback back over the serial connection. <br>
On the Host PC, the <i>Configurator</i> program (and later the driver) connects to the USB device and 2D Blobs are streamed in. These 2D Blobs are converted to 3D rays using the position of the cameras in the room and the poses of the markers are inferred. This is still in debate on how exactly this will work, there are several approaches that I will be trying out. <br>
In the Documentation you can find <a href="https://github.com/Seneral/MakersVR/tree/master/Documentation/Design">more detailed design documents</a>, alongside a <a href="https://raw.githubusercontent.com/Seneral/MakersVR/master/Documentation/PreliminaryPartsList.ods">preliminary parts list</a>. This places the currently expected costs between 110€ and 200€, depending on how many and which cameras are used.

## Current State
I currently have a two-camera setup working as a prototype, and it works functionally. However I have not had luck with accurate intrinsic calibration yet, which holds me up from going any further until I have that figured out. A lot of the prototype is of course not final, the <i>Marker Detector</i> operates on an old GL backend that I am working to replace with a much better backend in the near future, for example. <br>
Here's one of the <i>Marker Detector</i> with the <i>Marker Tracker</i>:
<p align="center">
  <img alt="Hardware Prototype of MakersVR" src="https://github.com/Seneral/MakersVR/raw/master/Documentation/Media/HW_Prototype_Annotated.jpg" width="50%"/>
  <br>
  The <i>Marker Detector</i> and <i>Marker Tracker</i> prototypes
</p>
The actual tracking system on the host PC is functionally done and has been developed and successfully tested in testing environments. However it might need some adjustments to work practically. <br>
The <i>Configurator</i> software currently contains a full testing mode and a device mode which connects to the actual hardware. In both modes you can do intrinsic calibration and extrinsic&room calibration phases using the calibration marker, as well as marker calibration and tracking phases using a tracking marker. <br> 
<p align="center">
  <img alt="Software Configurator of MakersVR" src="https://github.com/Seneral/MakersVR/raw/master/Documentation/Media/SW_Configurator_Annotated.png" width="60%"/>
  <br>
  The <i>Configurator</i> streaming blobs from the Prototype Hardware
</p>
The marker detection in the multi-camera marker tracking works similar to the <a href="https://ar-tracking.com/products/markers-targets/markers/">ART System</a>, with each 3D point identified based on the relations to its neighbours within the marker. This is marker configuration can be easily read in using a calibration phase. <br>
<p align="center">
  <img alt="Multi-Camera Tracking" src="https://github.com/Seneral/MakersVR/raw/master/Documentation/Media/SW_Multi-Camera-Tracking.gif" width="40%"/>
  <br>
  Simulated marker tracking of a WMR-controller-like tracker. 
</p>
Any marker that has LEDs with (relatively) unique distances works fine. But a cheap and easy to produce marker is a huge challenge. The WMR-like ring simulated above is way too expensive for home production in low quantities, so a more traditional design is used, a base supporting several sticks with LEDs at the tip (see the <a href="https://ar-tracking.com/products/markers-targets/targets/">ART targets</a> to get an idea). The base is going to be 3D-printed and houses power distribution (1S LiPo), and can be adapted to support many different marker layouts - however the current prototype is made out of polymorph plastic, which allows for even quicker prototyping, <a href="https://github.com/Seneral/MakersVR/raw/master/Documentation/Media/HW_Marker_PolymorphPlastic.jpg">as seen here</a>. The arms are made out of polypropylene straws, as they can survive quite a beating (for when you inevitably punch the walls) and can be easily replaced. The LED tips need to be spherical and well diffused, the current design is using a 3mm flat-top LED surrounded by sanded hot glue: <br>
<p align="center">
  <img alt="Marker Prototype" src="https://github.com/Seneral/MakersVR/raw/master/Documentation/Media/HW_Marker_Prototype.jpg" width="71.875%"/>
  <br>
  Marker Prototype powered by a 1S LiPo. Proportions and positioning can vary.
</p>
One hardware problem I'm still facing is the camera selection. The ecosystem of cameras for the Raspberry Pi is very limited, with the official modules having terribly low field of view, resulting in a smaller tracking volume, and most inofficial modules basing on the older camera module with much worse performance or a completely different chip alltogether with questionable support. Currently, I'm seeing if the Official Camera Module V2 with limited Field of View is viable: <br>
<p align="center">
  <img alt="Software Configurator of MakersVR" src="https://github.com/Seneral/MakersVR/raw/master/Documentation/Media/HW_CameraTrackingVolume.png" width="60%"/>
  <br>
  Tracking volume of three Marker Detectors using the Camera Module V2 in a small room
</p>

## Going Forward
Although having a technically fully functional first prototype, it is not ready for use yet, and there's a lot to improve still: <br>
1. Test out real hardware properly and improve accuracy along the way
2. Finish alternative QPU backend on the <i>Marker Detector</i> for improved performance and accuracy
3. Add additional CPU blob detection pass to increase accuracy 
4. Implement camera sync feedback (else fast movements will be inaccurate)
5. Add support for multiple properly tracked markers (currently simple frame-by-frame detection)
6. Adapt SteamVR driver (SerialHMD) to general tracker use for FBT
7. Develop power-management electronics to keep LiPo safe and develop PCB
8. Develop final marker design(s), create 3D-print base
9. Create a (3D-printed?) case for the electronics

Along the way, depending on interest, I will expand the documentation on this project. <br>

## Rebuilding
Although the software is not finalized yet, the first prototype can be build in its current state using just a soldering iron and it will stay the same for the forseeable future. All of the accuracy and performance improvements are purely software work. The required hardware components (with some parameters to adjust to budget) can be found <a href="https://github.com/Seneral/MakersVR/raw/master/Documentation/PreliminaryPartsList.ods">here</a>. If there's interest I'll add detailed instructions how to wire everything and tips for creating the calibration marker and tracking marker. <br>
The software already has all required instructions for compiling in the respective subfolders, especially the testing mode of the <i>Configurator</i> software can be used without hardware. 

## Sub-Projects
- Marker Detector: Software of the Raspberry Pi
  - <a href="https://github.com/Seneral/VC4CV">VC4CV</a>: The backends (GL and QPU) of the blob detection
- Marker Tracker: Software for the STM32F103 that interfaces with the Marker Detectors (UART /w DMA) and the Host PC (USB)
- Configurator: Test and future Configuration Software of the Host PC. Will be split later into tracking, configurator and driver

## Tracking Quality
All I can deliver about tracking quality right now are preliminary results. Here are some numbers for the current prototype: <br>
#### Latency
The newest WIP prototype uses the QPU backend of VC4CV, which is much, much faster than the GL backend that is currently in the repository.
- The camera processing time before reaching my code is unknown, but below 5ms
- The frametime is 2-3ms for a 640x480 image and 7-10ms for a 1640x1232 image - see below for more results.
- The UART connection is currently only 11520 baud, so another ~8ms is added for just three markers
- STM32 adds little to no latency, and considering the frame time and delay on host side, another 2-4ms can be added

This would add up to around 25ms, just to get it on the PC. <br>
For the final version, I hope to speed up the serial connection, or use additional differential hardware.
Assuming 1Mbaud (using either UART or SPI), it would only take around 1ms to transfer average data, so the final latency from movement to PC might drop as low as 10-15ms.

##### Frametime results
Results using WIP blob detection, from the moment the camera framework finishes a frame, to the moment the blobs are extracted.
There are still some optimizations possible in ALL steps, but mostly Fetch, as the information where the blobs are could be passed from the QPU instead.
Here's with a single blob with 10px radius:
```
Zero, 1536x1232: 142fps; QPU 3.68 + Fetch 2.70 + CCL 0.12 => 6.5ms
Zero, 640x480: 482fps; QPU 1.01 + Fetch 0.46 + CCL 0.12 => 1.59ms
Zero, 512x480: 575fps; QPU 0.77 + Fetch 0.37 + CCL 0.12 => 1.26ms
```
And here's with a single blob with 40px radius:
```
Zero, 1536x1232: 126fps; QPU 3.72 + Fetch 2.82 + CCL 0.79 => 7.33ms
Zero, 640x480: 345fps; QPU 1.02 + Fetch 0.59 + CCL 0.76 => 2.37ms
Zero, 512x480: 390fps; QPU 0.77 + Fetch 0.50 + CCL 0.75 => 2.02ms
```
QPU: Blob detection to mask on the QPU - Fetch: Fetching regions with blobs from the mask using CPU - CCL: Connected Component Labeling to extract distinct blobs using CPU

## Contact
Depending on what you want:
- Create an issue on this repository for involved technical discussions
- Catch me on Discord (Seneral#8110) for quick questions / discussions
- Send me an email otherwise (contact@seneral.dev)

## License
Refer to each individual sub-project (MakersVR_\*) for their licenses. Currently, all are licensed under MPL-2.0 <br>
The rest of the materials in this repository are licensed under the MIT license.

