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

## Functional Description
When the launch file is launched with the `target_colour` parameter, [beacon_search.py](scripts/beacon_search.py) subscribes to the colour camera topic, and begins to process a cropped view of the camera, performing a mask, effectively getting the number of pixels that are the specified colour. When a large enough amount are found (compared with the last time), a photo is saved to [target_beacon.jpg](snaps/target_beacon.jpg). To ensure the full width of the beacon is captured, the left and right of the image are checked to not contain too much of the specified colour.

Our application makes use of the SLAM toolbox for saving the map and for exploring the maze. SLAM constantly maps out the maze using the odometry and LIDAR sensor, and [map_saver_client.py](scripts/map_saver_client.py) saves this map every 5 seconds, by subscribing to the map saver in the package `nav2_map_server`.

[exploration.py](scripts/exploration.py) makes use of the SLAM toolbox by using the map of the area provided. While the robot is moving, it is constantly checking the LIDAR sensor, checking that there are no obstacles too close to the front of it. If there are, then it makes sure to avoid them by stopping and rotating. When no obstacles are too close in front, it navs to the nearest frontier. It uses occupancy grid ...........

### Functional Block Diagram
| ![Image of FBD](/FBD.png) |
| :--: |
| *Functional Block Diagram (FBD) of our application* |

## Contributors
* Andrew Bell - [GitHub](https://github.com/AndrewBell49)
* Nathan Hutchings - [GitHub](https://github.com/NathanHuttch)
