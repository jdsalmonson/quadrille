# quadrille

### Description

Quadrille is a ROS node that executes a sequence of dance moves at a given tempo and meter.  The dance is executed by publishing Twist commands to the cmd_vel topic, so should work for any robot that can move by subscribing to this topic.  If you are using the [PRSG2](http://programmingrobotsstudygroup.github.io/) robot running the [ros_arduino_bridge](https://github.com/ProgrammingRobotsStudyGroup/ros_arduino_bridge), two of the LEDs on the Pololu A-star 32U4 will blink to the tempo of the music: LED #1 on the first beat of each measure and LED #2 on subsequent beats.

To run (with a `roscore` running on your system), type:
```
rosrun quadrille quadrille.py
```

### Installation

Since this is a ROS node, you can install it in your catkin workspace.  Clone this repo into your `~/catkin_wd/src/` directory.  Then 
`cd ~/catkin_ws` and run `catkin_make` and `source devel/setup.bash`.  
