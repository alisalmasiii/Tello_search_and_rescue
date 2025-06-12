# `Tello Search and Rescue Project`

`tello_gazebo` required changes:
* First clone the `drone_racing_ros2` workspace from https://github.com/TIERS/drone_racing_ros2/tree/main. Then from this repository add these files keeeping the same folder structure.
* `launch` contains the required launch file to deploy the two drones
* `models` contains Gazebo models for AruCo markers, trees and grass.
* `search_tello.py` runs the search algorithms to detect the lost drone in the world.
* `lost_tello.py` runs the follower drone upon receiving communication from the search drone
* For this project, it is required to tilt the camera plugin (in radians - 0.6) to emulate the Gazebo tilted camera

  ## Installation
* Follow the guide from the TIERS git mentioned above
 
  #### Running the scenario
 ```bash  
  cd ~/drone_racing_ros2_ws
    source install/setup.bash
    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
    source /usr/share/gazebo/setup.sh
    ros2 launch tello_gazebo search_and_rescue.py
  ```
  
  #### Control the drones
  Run the python script
  ```bash
  python3 search_tello.py
```

  Then take-off drone1
  ```bash
    ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
```

  Run the second python script
  ```bash
python3 lost_tello.py
```
  Then take-off drone2
   ```bash
ros2 service call /drone2/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
```
