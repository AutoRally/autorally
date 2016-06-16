# AutoRally

![alt text](doc/autorally_repo.jpg "Platform image")

Software for the AutoRally research platform.

[AutoRally Platform Website](http://autorally.github.io)
[AutoRally Youtube Channel](https://www.youtube.com/channel/UCSt0P1uqi4zU5RX2DZC_Qvg)

Research Pages AutoRally is associated with:
  * http://rehg.org/autorally
  * http://dcsl.gatech.edu/research-muri-nascar.html
  * http://acds-lab.gatech.edu/Research.html


## Setup Instructions

### Contents
1. [Install Prerequisites](#1-install-prerequisites)
2. [Clone repository](#2-clone-repository)
3. [Compilation/Running](#3-compilationrunning)
4. [Generate Documentation](#4-generate-documentation)
5. [Test Setup in Simulation](#5-test-setup-in-simulation)
5. [What's Next](#6-what's-next)
5. [Contributing](#7-Contributing)

### 1. Install Prerequisites
1. __Install [Ubuntu 14.04 64-bit](http://www.ubuntu.com)__
2. __Install required packages__

   ```sudo apt-get install git doxygen openssh-server libusb-dev texinfo```
   
   _Recommended Tools_
   
   The following tools are useful, but not necessary for this project.
   * feh
   * cutecom
   * cmake-curses-gui
   * coriander
   * synaptic
   * arduino
   * python-termcolor
   
   _Compute Box Dependencies_
   
   When installing software on an AutoRally compute box, include these dependencies as well:
   
   ```sudo apt-get install lm-sensors acpi gpsd-clients```
   
3. __[Install](http://www.ros.org/install/) ros-indigo-desktop-full__
4. __Install gtsam__

   Follow the gtsam [Quick Start](https://bitbucket.org/gtborg/gtsam/) guide to clone and install the _develop_ branch of gtsam. 

   Instead of `cmake ..`, use:

   ```cmake -DGTSAM_INSTALL_GEOGRAPHICLIB=ON -DGTSAM_WITH_EIGEN_MKL=OFF ..```

   Once install is complete, make sure linux can see the shared library:

   ```sudo ldconfig```
   
### 2. Clone Repositories

Clone the autorally and imu_3dm_gx4 repositories into a [catkin workspace](http://wiki.ros.org/catkin/workspaces). The suggested location is `~/catkin_ws/src/`, but any valid catkin worskspace source folder will work.

```git clone https://github.com/AutoRally/imu_3dm_gx4.git```

```git clone https://github.com/AutoRally/autorally.git```

### 3. Install AutoRally ROS Dependencies

Within the catkin workspace folder, run this command to install the packages this project depends on.

```rosdep install --from-path src --ignore-src -y```

### 3. Compilation & Running

To compile and install run `catkin_make` from the catkin workspace folder.

Due to the additional requirement of ROS's distributed launch system, you must run `source autorally/autorally_util/setupEnvLocal.sh` before using any autorally components. For more information about how to set this system up for distributed launches on your vehicle platform, see the wiki.

_Note:_ If you are unfamiliar with catkin, please know that you must run `source <catkin_ws>/devel/setup.sh` before ROS will be able to locate the autorally packages. This line can be added to your ~/.bashrc file.

### 4. Generate Documentation

You can generate / update code documentation by running `doxygen` in `autorally/`.

To view code documentation open `autorally/doc/html/index.html` in a web browser.

### 5. Test Setup in Simulation

To test that your setup process was successful, run the AutoRally simulator with the following command. If you have a USB gamepad, plug it in to your computer before launching this simulator. You will be able to drive the virtual platform around using this gamepad.

```roslaunch autorally_gazebo autoRallyTrackGazeboSim.launch```

### 6. What's Next

Check out [our wiki](https://github.com/AutoRally/autorally/wiki) that includes:
* Instructions to configure a physical AutoRally platform
* Tutorials about controllers we release
* Tutorial to learn how to use your own controller with the AutoRally platform

### 7. Contributing

We welcome bug fixes, ehancements, new features, and [feedback](https://github.com/AutoRally/autorally/issues)!

Please submit pull requests to the devel branch that conform to the [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide).

We use Gitflow, so master branch is reserved for releases.