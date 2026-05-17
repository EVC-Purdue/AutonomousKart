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

- JETSON
```bash
sudo docker compose -f compose/docker-compose.yml up -d dev-jetson
```

- RUBIK
```bash
sudo docker compose -f compose/docker-compose.yml up -d dev-rubik
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

5) Launch

- JETSON
```bash
ros2 launch autonomous_kart bringup_jetson.launch.py
```

- RUBIK
```bash
ros2 launch autonomous_kart bringup_rubik.launch.py
```

- ALL
```bash
ros2 launch autonomous_kart bringup_all.launch.py
```

- SIM
```bash
ros2 launch autonomous_kart bringup_sim.launch.py
```