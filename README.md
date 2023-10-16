# Overview
This package is for integrating safety feature to the existing ROS2 system. 3 different inputs are available to trigger safety feature.
1. Digital output sensor (ex, Light curtain)
2. Analog output sensor (ex, Lidar)
3. Torque calculated from equation of motion

Although Moveit is used for stopping the robot in this package, you can modify this to use different method.

# Environment
Tested environment is on Ubuntu 22.04 with ROS2 humble.

# Prerequisite
As mentioned earlier, this package uses Moveit to stop the robot. In Moveit, there are 2 different ways to execute the path. One is asynchronous execution and the other is synchronous execution. Currently, only asynchronous execution can be stopped during the motion. Therefore, all the executions in your system should be asynchronous to make this package works as expected.

# Settings
There are 4 topics you can remap and 3 parameters you can adjust in launch file.

```
remappings=[
    ("joint_states", "joint_states"),
    ("required_torque", "required_torque"),
    ("digital_output", "digital_output"),
    ("analog_output", "analog_output")
],
parameters=[
    {"planning_group": "my_robot"},
    {"analog_threshold": 10},
    {"torque_diff_threshold": [10, 10, 10, 10, 10, 10]}
]
```

|topic|description|
|:---|:---|
|joint_states|topic where effort values are considered as actual torque values|
|required_torque|topic where effort values are considered as desired torque values|
|digital_output|output from sensor|
|analog_output|output from sensor|

|parameter|description|
|:---|:---|
|planning_group|name of your robot declared in URDF file|
|analog_threshold|threshold for analog output, if sensor output is higher than threshold, safety stop will be triggered|
|torque_diff_threshold|threshold for torque diff(actual torque - desired torque), should be the same number as the number of joint|

# Start up
Clone this repository into your ROS2 workspace and build the package.
```
colcon build --symlink-install
```

Modify topic names and prameters in `launch/safety_feature_launch.py` for your use case.

Start the node with launch file
```
ros2 launch safety_feature safety_feature_launch.py
```