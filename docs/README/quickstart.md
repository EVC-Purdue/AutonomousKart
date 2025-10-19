# QuickStart!

## Instructions

1) Run DevContainer (instructions in `docs/setup`)
2) Run **Once**
```bash
source install/setup.bash 
```
3) To build/test, run:
```bash
colcon build
ros2 launch autonomous_kart bringup_sim.launch.py
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
