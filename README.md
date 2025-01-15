# diffdrive_arduino

This node is designed to provide a ros2_control hardware interface for an Arduino running firmware from `ros_arduino_bridge`.
It is designed to be used with a `diff_drive_controller` from `ros2_control`.
It is expected to communicate via serial and to have two motors, each with velocity control and position/velocity feedback.


It is based on the diffbot example from [ros2_control demos](https://github.com/ros-controls/ros2_control_demos/tree/master/example_2).

For a tutorial on how to develop a hardware interface like this, check out the video below:

https://youtu.be/J02jEKawE5U


## Update by wty-yy
1. Change [diffbot_system.cpp](./hardware/diffbot_system.cpp) and [arduino_comms.hpp](./hardware/include/diffdrive_arduino/arduino_comms.hpp) to captiable with my Arduino board command in [arduino_pid_controlled_motor](https://github.com/wty-yy/arduino_pid_controlled_motor/)
2. Change default config file [diffbot.ros2_control.xacro](./description/ros2_control/diffbot.ros2_control.xacro) to fit my robot

## Usage
Install C++ serial library:
```
sudo apt install libserial-dev

cd ~/ros2_ws
colcon build --symlink-install
```
Use command below to test this drive
```bash
ros2 launch diffdrive_arduino diffbot.launch.py
```
