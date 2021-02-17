# navsat_datum_initializer

The navsat_datum_initializer package, based on RComponent structure. Node to call the datum service of navstat_transform_node.

The purpose of the package is to initialize the global position and orientation (specially) for the node *navsat_transform_node* in the [robot_localization](https://github.com/cra-ros-pkg/robot_localization) package. Although it has been tested on this [fork](https://github.com/RomanRobotnik/robot_localization/tree/kinetic-devel).

**How it works?**

The idea is:
* Move the robot forward (just linear x velocity).
* Save gps coordinates
* Estimate the linear equation by using linear regression
* Calculate the angle of the vector that goes from the initial coordinate to the last one.
* Set the datum into the navsat_transform_node


## Installation

Requires:
- [RComponent](https://github.com/RobotnikAutomation/rcomponent) package
- python-pip
- python2.7

The rest ones can be installed with rosdep on the workspace.

```
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
```

## 1 datum_initializer_node

Node in charge of performing the actions described above.

### 1.1 Parameters

* ~desired_freq (double, default: 10)
   Desired control loop frecuency
* ~gps_topic_name (string, default: 'gps/fix')
   Topic to read the current gps fix coordinates
* ~cmd_vel_topic_name (string, default: 'cmd_vel')
   Topic to send velocity commands to the robot
* ~base_frame (string, default: 'base_link')
   Base frame id. Used to check the distance traveled related to fixed_frame
* ~fixed_frame (string, default: 'odom')
   Fixed frame id
* ~set_datum_service_name (string, default: 'navsat_transform_node/datum')
   Service name to set the datum


### 1.2 Subscribed Topics

* gps/fix (sensor_msgs/NavSatFix)
  Gets the current gps fix coordinates

### 1.3 Published Topics

* cmd_vel (geometry_msgs/Twist)
  topic description, including any important rate information

### 1.4 Services


### 1.5 Services Called
* navsat_transform_node/datum (navsat_transform_node/SetDatum)
  Sets the global frame origin on navsat_transform_node

### 1.6 Action server
* action (navsat_datum_initializer/DatumInitializer)
  Triggers the measurement process and the estimation of the new datum

### 1.7 Action clients called


### 1.8 Required tf Transforms
* fixed_frame → base_frame

### 1.9 Provided tf Transforms


### 1.10 Bringup
