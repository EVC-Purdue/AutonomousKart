# QuickStart

## Start

1) Run DevContainer (instructions in `docs/setup`)
2) Run **Once**
```bash
source /opt/ros/humble/setup.bash
```
3) To build/test, run:
```bash
colcon build
ros2 launch autonomous_kart bringup_sim.launch.py
```

## Run devcontainer
1) Start docker
```bash
sudo systemctl start docker
```
2) Make container

`compose/docker-compose.yml` has one service per board (`jetson`, `rubik`, `pi`)
gated behind a matching Compose profile -- the profile you activate is what
decides which devices get mapped in (GPS/CAN/IMU on jetson, camera on rubik,
all four on a single-board `pi` setup) and which `scripts/kart` mode runs, so
there's no per-board editing of the compose file needed:
```bash
sudo docker compose -f compose/docker-compose.yml --profile pi up -d
# or: --profile jetson / --profile rubik for a distributed setup
# (or export COMPOSE_PROFILES=pi so plain `docker compose up -d` picks it up)
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
ros2 launch autonomous_kart bringup_pi.launch.py
```
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch autonomous_kart bringup_pi.launch.py

```bash
ros2 bag record -s mcap -o /ws/bags/$(date +%Y%m%d_%H%M%S) /mpc/status /mpc/residual_mode /mpc/residual_revert /cmd_drive /odom /gps /imu /imu/calibration_status /e_comms/kart_speed_m_per_s /e_comms/throttle_pwm /e_comms/steering_pwm /e_comms/adcb_state /e_comms/rc_mode /system_state /track_angles /manual_commands /pathfinder_params /pathfinder/active_planner
```
docker cp ros2-dev:/ws/bags .
