# QuickStart!

## Start

1) Run DevContainer (instructions in `docs/setup`)
2) Run **Once**
```bash
source install/setup.bash 
```
3) To build/test, run:
```bash
colcon build
ros2 launch autonomous_kart bringup_manual.launch.py
```

## Run devcontainer
1) Start docker
```bash
sudo systemctl start docker
```
2) Make container
```bash
sudo docker compose -f compose/docker-compose.yml up -d dev
```
3) Exec into container
```bash
docker exec -it <container_name> bash
```
4) Setup ROS
```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```
5) Setup ROS
```bash
source /opt/ros/humble/setup.bash
```

## Teams

### ROS Team:
1) Hot loop runs too slow!!!
2) Connector between opencv_pathfinder and pathfinder is slow. Debug!

### OpenCV Team:
1) Implement solution in `autonomous_kart/nodes/opencv_pathfinder/angle_calculator.py`
2) Add efficient error checking

Note: Ensure this code is efficient as it is in the hot loop

### Pathfinding Team:
1) Implement solution in `autonomous_kart/nodes/pathfinder/pathfinder.py`
2) Add efficient error checking

Note: Ensure this code is efficient as it is in the hot loop

### 3DGS Team:
1) Fix library :(
2) Think about optimizations to localization

Note: Not critical until after Thanksgiving break


## Info

1) Hot loop runs at ~60 Hz
2) Error checking not implemented (**YET!**)

## MVP
Drive successfully around the track and stay between lines

**Nodes:** camera, opencv_pathfinder, pathfinder, motor, steering\
**Unused:** gps, 3dgs_localization
