# SoftClaw node
ROS node to communicate with SoftClaw Device

## Installation
### Requirements
If you have never set it up, you probably need to add your linux user to the `dialout` group to grant right access to the serial port resources. To do so, just open a terminal and execute the following command:
```
sudo gpasswd -a <user_name> dialout
```
where you need to replace the `<user_name>` with your current linux username.

_Note: don't forget to logout or reboot._

### Sources
>Since you are interested in the ROS interfaces for our devices, it is assumed that you are familiar at least with the very basics of the ROS environment. If not, it might be useful to spend some of your time with [ROS](http://wiki.ros.org/ROS/Tutorials) and [catkin](http://wiki.ros.org/catkin/Tutorials) tutorials. After that, don't forget to come back here and start having fun with our Nodes.

Install the package for a ROS user is straightforward. Nonetheless the following are the detailed steps which should be easy to understand even for ROS beginners:

1. Clone the package to your Catkin Workspace, e.g. `~/catkin_ws`:

1. Compile the packages using `catkin`:
   ```
   cd `~/catkin_ws`
   catkin_make
   ```

### Device Setup
Connect the device to your system through the USB connection and power up the system with the proper voltage [24V for SoftClaw].

## Configure
Main parameters can be configurable by a YAML file, which is request during the call to softclaw node with "roslaunch" method.
If you do not specify your yaml own custom file, by variable yamlFile:="PathToYourConfFile", a default yaml fill will be set.

example

**communication port**

port: '/dev/ttyUSB0'

**ID of the device**

ID: 1

**Node frequency [Hz]** 

run_freq: 100

**Reference commands topic**

ref_claw_topic: "ref"

**Actual position reading topic**

meas_claw_topic : "meas"

## Communication
With default configuration, communication is allowed through two topics.

  - /SoftClaw/ref
  - /SoftClaw/meas
  
**Message Type**

[/SoftClaw/ref]

Message is of type std_msgs::Float64MultiArray. First value is for position reference (in mm), while second value is for stiffness (in %).
Allowed values ranges are [0-110 mm] for position and [0-100 %] for stiffness.

[/SoftClaw/meas]

Message is of type std_msgs::Float64MultiArray. First value is the actual position of the claw in mm.
Expected values are in the range [0-110 mm].

## Basic Usage

Launch the node

```
$ roslaunch softclaw softclaw.launch
```

Send the commands through ROS topic (e.g. for 50mm position, 80% stiffness):

```
$ rostopic pub /SoftClaw/ref std_msgs/Float64MultiArray "layout:
dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [50.0, 80.0]"
```

