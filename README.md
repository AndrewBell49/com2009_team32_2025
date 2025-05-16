# COM2009 Team 32 Assignment 2

## Overview
Task 3 is an application, which controls the WaffleBot to navigate a maze. It uses its camera to take a photo of a beacon of a specified colour, and uses SLAM to map out and save a map of the maze. During the running of the application, the robot aims to explore as much of the maze as possible.

## Installation and Execution
If copying fresh, you may need to check that the required scripts are executable. In the package directory, run:
```bash
cd scripts
ls -l

```
beacon_search.py, exploration.py, and map_saver_client.py should all have an 'x' at the end of their permissions
If not, for each file, run the following:
```bash
chmod +x file_name.py
```
Then run:
```bash
cd ~/ros2_ws/ && colcon build --packages-select com2009_team32_2025 --symlink-install
source ~/.bashrc
```