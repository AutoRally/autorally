# AutoRally

![alt text](doc/autorally_repo.jpg "Platform image")

Software for the AutoRally research platform.

[AutoRally Platform Website](http://autorally.github.io)

[AutoRally Youtube Channel](https://www.youtube.com/channel/UCSt0P1uqi4zU5RX2DZC_Qvg)

Research Pages AutoRally is associated with:
  * http://rehg.org/autorally
  * http://dcsl.gatech.edu/research-muri-nascar.html
  * http://acds-lab.gatech.edu/Research.html

## Contributing

We welcome bug fixes, ehancements, new features, and [feedback](https://github.com/AutoRally/autorally/issues)!

Please submit pull requests to the [devel branch](https://github.com/AutoRally/autorally/pull/new/devel) that conform to the [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide). We use Gitflow, so master branch is reserved for releases.

## Setup Instructions

### Contents
1. [Install Prerequisites](#1-install-prerequisites)
2. [Clone repository](#2-clone-or-fork-repositories)
3. [Install AutoRally ROS Dependencies](#3-install-autorally-ros-dependencies)
4. [Compilation/Running](#4-compilation-running)
5. [Generate Documentation](#5-generate-documentation)
6. [Test Setup in Simulation](#6-test-setup-in-simulation)

### 1. Install Prerequisites
1. __Install [Ubuntu 16.04 64-bit](http://www.ubuntu.com)__
2. __Install required packages__

   ```sudo apt-get install git doxygen openssh-server libusb-dev texinfo```
   
   _Recommended Tools_
   
   The following tools are useful, but not necessary for this project.
   * feh
   * cutecom
   * cmake-curses-gui
   * coriander
   * synaptic
   * arduino (latest version of [Arduino IDE](https://www.arduino.cc/en/Main/Software))
   * python-termcolor
   
3. __[Install](http://www.ros.org/install/) ros-kinetic-desktop-full__
4. __Install gtsam__

   Follow the gtsam [Quick Start](https://bitbucket.org/gtborg/gtsam/) guide to clone and install the _develop_ branch of gtsam. 

   Instead of `cmake ..`, use:

   ```cmake -DGTSAM_INSTALL_GEOGRAPHICLIB=ON -DGTSAM_WITH_EIGEN_MKL=OFF ..```

   Once install is complete, make sure linux can see the shared library:

   ```sudo ldconfig```
   
### 2. Clone or Fork Repositories

Get the autorally repository in a [catkin workspace](http://wiki.ros.org/catkin/workspaces). The suggested location is `~/catkin_ws/src/`, but any valid catkin worskspace source folder will work. We suggest forking first if you will be working with the code.


To clone straight from the AutoRally repo:

    git clone https://github.com/AutoRally/autorally.git

Also clone the IMU code into the same catkin workspace:

    git clone https://github.com/AutoRally/imu_3dm_gx4.git

### 3. Install AutoRally ROS Dependencies

Within the catkin workspace folder, run this command to install the packages this project depends on.

```rosdep install --from-path src --ignore-src -y```

### 4. Compilation & Running

To compile and install run `catkin_make` from the catkin workspace folder.

Due to the additional requirement of ROS's distributed launch system, you must run

`source src/autorally/autorally_util/setupEnvLocal.sh`

before using any AutoRally components. See the [wiki](https://github.com/AutoRally/autorally/wiki) for more information about how to set this system up for distributed launches on your vehicle platform.

_Note:_ If you are unfamiliar with catkin, please know that you must run `source catkin_ws/devel/setup.sh` before ROS will be able to locate the autorally packages. This line can be added to your ~/.bashrc file.

### 5. Generate Documentation

You can generate or update code documentation by running `doxygen` in `autorally/`.

To view code documentation open `autorally/doc/html/index.html` in a web browser.

### 6. Start the AutoRally Simulation to Test Configuration

```roslaunch autorally_gazebo autoRallyTrackGazeboSim.launch```

You can use a USB gamepad to drive the simulated platform around. On startup, the `runstop` message published by the `joystick` node is **false**. Press any of the buttons by the right stick (normally labelled X, Y, A, B or square, triangle, X, circle) to toggle the published value.

Verify runstop motion is enabled by looking at the `runstopMotionEnabled` paramter in the `/chassisState` topic.

If you aren't using a gamepad, you will have to configure another source of runstop information for the platform to move:

- Comment out line 93 of `autorally_gazebo/launch/autoRallyTrackGazeboSim.launch`

- ```rosrun rqt_publisher rqt_publisher```

and configure rqt_publisher to publish a message to topic `/runstop` of type `autorally_msgs/runstop` at 1 Hz with `sender` set to `rqt_publisher` and  `motionEnabled` set to **true**.

- Verify that `runstopMotionEnabled` is **true** in `/chassisState` topic.

### 7. Autonomous Driving in Simulation

At the end of this section the robot will be driving autonomously in simulation using controllers available in `autorally_control`.

Position the robot in the same spot as when the simulation starts and make sure runstop motion should is enabled (set to **true**).

#### Start state estimator:

In `autorally_core/launch/state_estimator.launch` change `InvertY` and `InvertZ` to **false**, then:
    
    rosparam set /gps_imu/FixedInitialPose true
    roslaunch autorally_core state_estimator.launch

#### Start waypoint follower:

    roslaunch autorally_control waypointFollower.launch

#### Start constant speed controller and tell it what speed to drive:

    roslaunch autorally_control constantSpeedController.launch
    rosrun rqt_publisher rqt_publisher

Configure a publisher on topic `constantSpeedController/speedCommand` of type `std_msgs/Float64` at rate 10 with value of 3 (you can adjust he value once everything is running). The value is the target velocity in m/s, and **as soon as you do this the platform should move if motion is enabled**.

If the robot turns and hits the barrier it's probably because the state estimator has not converged, so its orientation estimate is incorrect. Just select the track barriers and move them up to allow the robot to continue driving, and the estimator should converge and the vehicle will return to within the barriers.

## What's Next

More detailed explanations of the controllers and state estimator can be found on the [wiki](https://github.com/AutoRally/autorally/wiki):
* [State estimator](https://github.com/AutoRally/autorally/wiki/State%20Estimator)
* [Waypoint follower](https://github.com/AutoRally/autorally/wiki/Waypoint%20Following)
* [Constant speed controller](https://github.com/AutoRally/autorally/wiki/Constant%20Speed)

[Controlling the AutoRally platform](https://github.com/AutoRally/autorally/wiki/Controlling%20the%20AutoRally%20Platform) is a tutorial for how your own controller can control the AutoRally platform (in simulation or on hardware).

If you are configuring a physical AutoRally platform, the next step is to configure the compute box, all of the peripherals, and the launch system. Those instructions are found in the [Platform Configuration Instructions](https://github.com/AutoRally/autorally/wiki/Platform%20Configuration%20Instructions).
