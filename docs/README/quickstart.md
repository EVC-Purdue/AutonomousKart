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

## Info

1) Hot loop runs at ~60 Hz
2) Error checking not implemented (**YET!**)

## MVP
Drive successfully around the track and stay between lines

**Nodes:** camera, opencv_pathfinder, pathfinder, motor, steering\
**Unused:** gps, 3dgs_localization
