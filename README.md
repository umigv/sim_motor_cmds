# Synopsis
Simulation of motor control through turtlesim. It takes in a path of coordinates to perform trajectory planning and publish
motor commands.

# Motivation
We need a more efficient testing evnironment, instead of launching the full simulation with Gazebo and sensors, turtlesim is
suitable for this goal.

NOTE: the motion controls assume points are always in fron of robot. i.e. 
the turning angle is between -pi/2 to pi/2
Precision is also not perfect to simplify logic 
robot is considered reached target when x matches (y is not compared)

# Run
`roscore`

`rosrun turtlesim turtlesim_node`

`rosrun sim-motor-cmds sim-motor-cmds_node <path.txt>`

# Contributors
Authored and maintain by Elton Lin.


