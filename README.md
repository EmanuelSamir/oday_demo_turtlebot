# oday_demo_turtlebot 2020
Simple package to control the real turtlebot3 + arm manipulation. Scripts made were presented in OpenDay 2020-2.

First, connect remotely to the turtlebot3 using ssh protocol.

~~~
ssh pi@$turtlebot_ip
~~~

For our robots, `turtlebot_ip` could be 192.168.0.201 or  192.168.0.202 and default password is `turtlebot`.

Then, in local PC run

~~~
roscore
~~~

and in remote Turtlebot3, run

~~~
roslaunch turtlebot3_bringup turtlebot3_robot.launch 
~~~

Wheel control is now available. Further commands must be run. Arm Manipulator is controlled based on MoveIt package. Turtlebot3 provided necessary packages.

Run in PC:

~~~
roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch 
roslaunch turtlebot3_manipulation_moveit_config move_group.launch
~~~

Package is composed of three scripts  `src/arm_test.py`, `src/wheels_test.py`,  and `src/demo_test.py`. The one to be used is `src/demo_test.py`. It has four functions `set_arm_pose` to drive the arm to a defined euclidean position. `set_gripper` to control the openness.

