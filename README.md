# COM2009 Team 32 Assignment 2

## Overview
Task 3 is an application, which controls the WaffleBot to navigate a maze. It uses its camera to take a photo of a beacon of a specified colour, and uses SLAM to map out and save a map of the maze. During the running of the application, the robot aims to explore as much of the maze as possible.

## Installation and Execution
If copying fresh, you may need to check that the required scripts are executable. In the package directory, run:
```bash
cd scripts
ls -l
```
`beacon_search.py`, `exploration.py`, and `map_saver_client.py` should all have an 'x' at the end of their permissions
If not, for each file, run the following:
```bash
chmod +x file_name.py
```
Then run:
```bash
cd ~/ros2_ws/ && colcon build --packages-select com2009_team32_2025 --symlink-install
source ~/.bashrc
```

### Dependancies
* `rclpy`
* `cv2`
* `cv_bridge`
* `sensor_msgs`
* `com2009_team32_2025_modules`
    * custom package, including a formula for angles
* `geometry_msgs`
* `nav_msgs`
* `nav2_msgs`
* `sensor_msgs`
* `math`
* `numpy`
* `random`
* `os`

## Functional Description
When the launch file is launched with the `target_colour` parameter, [beacon_search.py](scripts/beacon_search.py) subscribes to the colour camera topic, and begins to process a cropped view of the camera, performing a mask, effectively getting the number of pixels that are the specified colour. When a large enough amount is found (compared with the last time), a photo is saved to [target_beacon.jpg](snaps/target_beacon.jpg). To ensure the full width of the beacon is captured, the left and right sides of the image are checked so as not to contain too much of the specified colour.

Our application makes use of the SLAM toolbox for saving the map and for exploring the maze. SLAM constantly maps out the maze using the odometry and LIDAR sensor, and [map_saver_client.py](scripts/map_saver_client.py) saves this map every 5 seconds, by subscribing to the map saver in the package `nav2_map_server`. The resulting image of the map will be saved to [arena_map.png](maps/arena_map.png)

[exploration.py](scripts/exploration.py) makes use of the SLAM toolbox by using the map of the area provided. When no obstacles are too close in front, it navigates to the nearest frontier. To get the nearest frontier, it makes use of the occupancy grid. It subscribes to /map and retrieves the visualised map from the lidar sensors. This visualised map is retrieved as an array of 0’s, 100’s, and -1’s where each one represents free space, walls, and unexplored space respectively. It then filters out and takes the -1’s as frontiers, which goes through a loop to choose the closest frontier, with paramaters put in place so if the frontier is within 1 meter of the robot, then discard it and look for the closest one that is more than a meter way. First it check if there is a wall infront, calculated using the LIDAR sensor, taking an average of the minimum 2 points, in the front 30 degrees of the robot. If there is, then it stops, rotates and chooses to go the direction of the goal. But we also added a bit of randomness. There is also a 33% chance that it does not rotate towards the goal. This decision was made after we saw for ourselves that the robot could get stuck in a room trying to get out to reach the next closest frontier. Moreover, when it is going to a frontier, it calculates the difference of yaw angle between the robot and where the goal is, and then it moves with an angle to the frontier. Once it is within 0.3 meters from the frontier, it takes it as a sign that the frontier is now reached and explored and searches for the next one.

### Functional Block Diagram
| ![Image of FBD](/FBD.png) |
| :--: |
| *Functional Block Diagram (FBD) of our application* |

## Contributors
* Andrew Bell - [GitHub](https://github.com/AndrewBell49)
* Nathan Hutchings - [GitHub](https://github.com/NathanHuttch)
* Moaz Elleithy - [GitHub](https://github.com/Moaz-Elleithy)
* Edward Dale - [GitHub](https://github.com/Eduardo-3541)