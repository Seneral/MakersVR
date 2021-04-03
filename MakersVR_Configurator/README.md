# Configurator

This is the configurator software running on the host PC (Win 10 or Linux 64 Bit) that communicates with the tracking system hardware and is responsible for calibration and setup as well as testing without hardware.

## Installation

No binaries are available yet, so you'll have to build it yourself. But the build systems are pretty fleshed out at this point so it should be pretty straightforward.

### Requirements
**For Windows 10:** MSVC compiler + nmake </br>
If you have Visual Studio installed, you're good </br>
Otherwise, to download the minimal setup (only the build tools):
  - Download the <a href="https://visualstudio.microsoft.com/visual-cpp-build-tools/">Visual Studio Build Tools</a>
  - On installation, select "C++ build tools"
  - Then select the following components on the right (can unselect the others):
    - `MSVC`, `Windows 10 SDK`, `C++ CMake`

Also make sure that <a href="https://docs.microsoft.com/en-us/cpp/build/building-on-the-command-line?view=msvc-160#developer_command_file_locations">vcvars64.bat</a> is executable:
  - Add it to the path, e.g: `C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\VC\Auxiliary\Build`
  - Alternatively, modify .vscode/tasks.json to point to the vcvars64.bat file

**For Linux:** </br>
 Standard build tools (`gcc`, `g++` and `cmake`) </br>
 OpenGL development libraries (`libgl1-mesa-dev`, `libglu1-mesa-dev`, `libglew-dev`) </br>

**Optional:** Visual Studio Code with C++ extension for development

### Dependencies
To create a minimal build of the dependencies I provided custom build scripts that try to only build the required modules (as OpenCV and wxWidgets are very large). </br>
Note none of these are installing any systemwide files, after the clean operation the only changed files are in the projects include/ and lib/ folders. </br>
In each individual dependencies/* directory execute the corresponding scripts: </br>
(Note *.bat is for Windows and *.sh for Linux)
  - (Windows) Make sure you are in vcvars64 environment of MSVC
  - `fetch.[sh/bat]`
  - `build.[sh/bat] source [release/debug]`

Then, if it succeeded:
  - `clean.[sh/bat] source`

### Compilation
While in the main configurator folder:
  - (Windows) In vcvars64 environment: `nmake -F Makefile.vc`
  - (Linux) `make`

Or if you use VS Code, just execute the apropriate build task (see .vscode/tasks.json)

### Linux Permissions
To make sure the program has access to the usb device (on recent ubuntu):
- `sudo echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="16c0", ATTR{idProduct}=="05df", MODE="0660", GROUP="plugdev"' >> /etc/udev/rules.d/00-usb-permissions.rules`
- `udevadm control --reload-rules`
- Assuming you and thus the program is in the group plugdev, if not:
- `sudo adduser $USER plugdev`
- and then restart (or log out and back in)

Alternatively, execute the program with sudo (not recommended)

### Notes
Both build and debug configurations work fine, using the respective builds of the dependencies. By default, dependencies are built in release mode, pass "debug" to build scripts to build dependencies in debug mode </br>
However, the build system is not perfect, in order to have changing headers cause all the scripts relying on this header to recompile, the headers are added manually as dependencies in both Makefiles, so they have to be kept in sync or weird stuff could happen.

## Testing
Here are some instructions for using the testing framework to test calibration using single-camera pose estimation and tracking using multi-camera triangulation. This does NOT require you to set up any of the hardware.

First, start testing by selecting `Testing/Start Testing`. Multiple simulated cameras frames will appear. Now you can start any phase from the leftmost dropdown (Idle by default).

The calibration has several phases and all you have to do is select the focus of the calibration while the calibration marker is randomly moved around in a way that should be optimal for the respective phase.

### Intrinsic Calibration
Start Intrinsic Calibration by selecting `Intrinsic Calibration` from that leftmost dropdown. </br>
Now you can choose the camera to focus on with the numbers 1-3.
The marker will be held at varying positions close to the camera, and once a certain amount of positions on the screen are covered, it will start a calibration round. </br>
During a calibration round, you can switch to other cameras, as it will not add any marker poses to the selection while a calibration round is ongoing for that camera. </br>
After a calibration round finishes, bad markers are removed and you can add even more markers, until the next calibration round starts.
You can see the calibration results compared to the ground truth after each calibration round for that camera - the blue dots are an undistorted grid, the green dots represent the ground truth distortion (only for testing), and the yellow dots represent the currently estimated distortion. </br>
Once you're happy with all cameras, press `Accept` to save the intrinsic calibration results to `calib.json`. Or press `Discard` to discard the current results, reusing existing calibration (or none if none existed).

### Extrinsic Calibration
Extrinsic Calibration consists of two phases - in the first, the calibration marker will try to face two cameras at once, so that their relative transform can be estimated. </br>
After selecting `Extrinsic Calibration` from the leftmost dropdown, choose the camera relation to focus on using the numbers 1-3 again.
Watch each relation get more accurate - this is calculated using ground truth, although obviously that is not used in the actual calibration. </br>
This time, press `Next` to continue with the second phase or `Discard` to abort extrinsic calibration. </br>
In this phase the camera origin is estimated, so that when relations and origin transforms are combined, all paths between cameras are weighted according to weakest link and transforms combined, so that all cameras will get an absolute reference frame. </br>
Just wait until all the origin transforms have a decent weight of around 1 or less, and then press `Accept` to save the extrinsic calibration results to `calib.json`. Or press `Discard` to discard the current results, reusing existing calibration (or none if none existed).

### Marker Calibration
Here, a 3D Marker will get calibrated - the system does not know it's shape but learns it by observing it. </br>
Choose any 3D marker from `Testing/Tracking Markers` and start by selecting `Marker Calibration` from the leftmost dropdown. </br>
Although this is not very fleshed out yet, you should see it building a model of some points in the center, and the points should get reinforced over time, while trying to remove invalid noise points. </br>
After a while, press `Accept` to save the marker calibration results to `calib.json`. Or press `Discard` to discard the marker.

### Tracking
As the name suggests, it's just tracking whatever 3D marker you chose from `Testing/Tracking Markers`. Start by selecting `Marker Calibration` from the leftmost dropdown. </br>
This requires proper calibration, if the markers aren't detected, load the backup calibration or recalibrate properly.