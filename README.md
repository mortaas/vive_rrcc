# vr_ros

vr_ros is a Robotic Operating System (ROS) package that utilises the [OpenVR SDK](https://github.com/ValveSoftware/openvr) by Valve, to make VR devices such as the HTC VIVE available in a ROS environment.

## Features

The package supports the following types of devices:
* HMD (Head-Mounted Display)
* Controller
* Tracker
* Lighthouse

It exposes the position and orientation of each device as coordinate frames relative to *world_vr* frame (one of the lighthouses) in the tf tree, which is configurable relative to the *world* frame. The naming scheme of each coordinate frame follows the following structure: &lt;device type&gt;_&lt;serial number&gt;, e.g. *controller_LHR_FF6FFD46*. This results in a structure similar to the tf tree example that is shown below:

![Example of the tf tree structure that is used in this package](frames.png)

It also publishes the linear and angular velocities (twists) of tracked devices as a geometry_msgs/TwistStamped message on the */vr/twist/&lt;device type&gt;_&lt;serial number&gt;* topic, e.g. */vr/twist/controller_LHR_FF6FFD46*. Axes and buttons on controllers is also published as a sensor_msgs/Joy message on the */vr/joy/&lt;device type&gt;_&lt;serial number&gt;* topic, e.g. */vr/joy/controller_LHR_FF6FFD46*.

## Requirements

## OpenVR SDK

The package requires the [OpenVR SDK](https://github.com/ValveSoftware/openvr), which has to be built from the newest available source. It is possible to download and build the source in the correct folder by utilising the following commands:
```
cd ~
mkdir lib
cd lib
git clone https://github.com/ValveSoftware/openvr.git
cd openvr
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ../
make
```

It is also possible to specify which folder the [OpenVR SDK](https://github.com/ValveSoftware/openvr) should be located in, by changing the *CMakeLists.txt* file in the package directory:
```
set(OPENVR "$ENV{HOME}/lib/openvr")
```

### Steam and SteamVR

SteamVR is available through [Steam](https://store.steampowered.com/about/), which is utilised for configuration and room setup. It is also required for running this package by itself, as it depends on the *vrserver* process running in the background. This is a requirement because the OpenVR part of the package runs as a background application ([OpenVR API Documentation](https://github.com/ValveSoftware/openvr/wiki/API-Documentation)):

```VRApplication_Background``` - The application will not start SteamVR. If it is not already running the call with VR_Init will fail with VRInitError_Init_NoServerForBackgroundApp.

[Steam](https://store.steampowered.com/about/) is installed by following the *Getting Started* guide on their [Steam for Linux](https://github.com/ValveSoftware/steam-for-linux) tracker. SteamVR should be installed automatically by Steam if there is any VR devices present on your computer. It is also important to meet the **GRAPHICS DRIVER REQUIREMENTS** and the **USB DEVICE REQUIREMENTS** on their [SteamVR for Linux](https://github.com/ValveSoftware/SteamVR-for-Linux) tracker. A complete guide on getting the HTC VIVE up and running in SteamVR is available from: [HTC Vive Installation Guide](https://support.steampowered.com/steamvr/HTC_Vive/).

## Installation

The package is built by cloning this repository into your catkin workspace (/src directory) and then making it with ```catkin_make```.

## Usage

The package is simply run by launching the following launch file:
```roslaunch vr_ros vr.launch```

## Compatibility

The package was tested with:
* OpenVR SDK 1.0.15 and Ubuntu 16.04 LTS running ROS Kinetic Kame (1.12.13)