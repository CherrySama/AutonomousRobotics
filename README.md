<<<<<<< Updated upstream
# AutonomousRobotics🤖
 
=======
# AutonomousRobotics

>>>>>>> Stashed changes
AutonomousRobotics Courseworks Shared purely for education purposes

📌Before running the codes, please make sure that **you've already created and initialized a working space.**

## Minitask1

The robot moves in a square with different feedback algorithm

| Step | Terminal | Commands                                            |
| ---- | -------- | --------------------------------------------------- |
| 1    | T1       | roscore                                             |
| 2    | T2       | roslaunch turtlebot3_gazebo turtlebot3_world.launch |
| 3    | T3       | rosrun minitask1 square_move_controller.py          |

## Minitask2

1 After going straight for 1 meter, turn left and right at will and continue walking for 1 meter at random.

2 After detecting the wall, go straight along the right wall.

3  When obstacles are detected in front of you, rotate to avoid obstacles.

| Step | Terminal | Commands                                        |
| ---- | -------- | ----------------------------------------------- |
| 1    | T1       | roscore                                         |
| 2    | T2       | roslaunch minitask3 turtlebot3_corridors.launch |
| 3    | T3       | rosrun minitask2 obstacle_avoidance.py          |

## Minitask3

Use the robot camera to look for the green object in front of you, move towards the green object, and stop in advance to the target object. Avoid obstacles encountered during exercise.

| Step | Terminal | Commands                                        |
| ---- | -------- | ----------------------------------------------- |
| 1    | T1       | roscore                                         |
| 2    | T2       | roslaunch minitask3 turtlebot3_corridors.launch |
| 3    | T3       | rosrun minitask3 follower.py                    |

## Minitask4

Navigation and Mapping (using waypoints to explore the space)

| Step | Terminal | Commands                                                     |
| ---- | -------- | ------------------------------------------------------------ |
| 1    | T1       | roscore                                                      |
| 2    | T2       | roslaunch turtlebot3_gazebo turtlebot3_house.launch          |
| 3    | T3       | roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=\`rospack find minitask4\`/maps/mymap.yaml |
| 4    | T4       | roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch     |
| 5    | T5       | rosrun minitask4 patrol.py                                   |

## Minitask5

1 Within a limited time, let the robot find as many red and green target objects as possible in each room of the map.

2 When you find the object, you need to print a message, for example: 'Find green object 1'. And to be marked on the Rviz map.

3 The blue floor tile in the map needs to be avoided and cannot be passed from above (this code is not implemented).

4 The robot cannot collide during driving and needs to successfully avoid obstacles.

| Step | Terminal | Commands                                                     |
| ---- | -------- | ------------------------------------------------------------ |
| 1    | T1       | roscore                                                      |
| 2    | T2       | roslaunch minitask5 train_env.launch                         |
| 3    | T3       | roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=\`rospack find minitask5\`/maps/train_env.yaml |
| 4    | T4       | rosrun minitask5 main.py                                     |
