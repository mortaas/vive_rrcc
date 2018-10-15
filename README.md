# Rapid Robotcell Calibration (RRCC)

This project is currently split into three [ROS](http://www.ros.org/) packages:
* vive_bridge: Main package for interfacing [HTC VIVE](https://www.vive.com/eu/) devices with [ROS](http://www.ros.org/)
* vive_calibrating: Tools for calibrating the vive_bridge package to a robot frame or an arbitrary reference frame
* vive_planning_scene: Simple tests for calibrating the position and orientation of an object in space

## Project description

HTC Vive is quite famous for their new virtual reality gaming headset. It has quickly become one of the best in the market. One of the reasons for its success was the innovative technology for tracking users hands and head. This tracking system consists of three or more “lighthouses” placed around the room. They sweep beams of light at a very precise frequency, and high-speed light sensors in the headset and remotes pick these up. Using these light signals, the system is capable of tracking the location of the remotes at a rate of 60 fps at an accuraccy of 0.3 mm in practically any size room.

In many industrial automation cases, a robot work cell must be calibrated such that the location of tools, surfaces, and parts are known to the robot. This can be done by jogging the robot (moving it by the remote controller), or by guiding the robot by hand (when the robot is in compliant mode), to specific setpoints in the work cell and recording their location. Another way is to construct the work cell with a high-degree of positional accuracy. In general for robots performing many tasks, and visiting many locations, this is time consuming and requires the robot to be installed ahead of time, or expensive and laborious. This can become very complicated when working with mobile robots that change location on the shop floor, or when the company desires to change setups quickly and often.

The work will be focused on interfacing with ROS: robotic operating system. At its core it is a middleware solution that implements a set of standards for communication between process. It’s been used by a variety of researchers and programmers around the world to control anything from industrial robots, to drones, to unmanned boats. There is a wealth of packages for visualization and control and many others are researching integration of ROS and HTC Vive, giving ample opportunity to get help and interest in your work.

~~In this master thesis~~ the student is to explore the use of an HTC Vive for rapid robot cell calibration. Can this technology let us map existing parts of a work cell faster? Can we visualize potential future work cells or alterations to the work cell with new components?

## Tasks
1. Set up HTC Vive for use with Ubuntu
2. Establish an overview of existing Vive-ROS bridging attempts34
3. Set up a ROS Node for publishing HTC Vive sensor data
4. Evaluate accuracy of handheld remotes for use with calibration

## Further work
There are three main directions that the project can branch into. The first is live-tracking of objects. If the accuracy and rate is sufficient with simply the handheld remotes it may serve as a ground truth for localization algorithms developed for drones. This would require construction of small, simple sensors that mimic the behavior of the handheld remotes.

The second is geometrics. If the accuracy is sufficient, the next step in a rapid robot cell calibration system would be to prototype software capable of placing real-world items in a virtual robot cell with their 3D models. The software would go from virtual task description, rapid calibration, to simulating a robotic assembly, and eventually performing a robotic assembly.

The third is calibration of Thrivaldi. Recently the Department of Engineering Cybernetics acquired a robotics lab from SINTEF. The lab consists of two KUKA robots, one hanging from a large gantry crane, and the other situated on the floor. This lab has been set up with a forward kinematics based on its 3D model, and if the Vive is sufficiently accurate, we can revise it and improve the robot description.