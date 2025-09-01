# liorf: LIO-SAM-6-axis working with ROS2
Repo to run LIO-SAM in docker with ROS2 Foxy and 6 axis IMU

Clone the repo, then build and enter the docker:

```
cd liorf
git checkout liorf-ros2
cd docker
docker build -t liosam_foxy .
xhost +local:docker
sudo chmod -R 777 container_run.sh
./container_run.sh liosam_foxy liosam_foxy:latest
cd ~/ros2_ws
colcon build --parallel-workers 2
source install/setup.bash
ros2 launch liorf run_M2DGR.launch.py
```

If this gives an error you need to stop and remove the previous image:
```
docker stop liosam_foxy
docker rm liosam_foxy
```

To adapt the script to your available topics, you have to change the `config` in the liorf repo


