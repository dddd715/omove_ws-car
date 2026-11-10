# gazebo
```bash
ros2 launch omove_description world01_sim.launch.py \
  slam:=false \
  gui:=true \
  x:=0.0 \
  y:=-1.0
```
# slam_toolbox定位(localization定位模式)
```bash
ros2 launch slam_toolbox localization_launch.py \
  use_sim_time:=true \
  slam_params_file:=/home/deng/omove_ws/src/omove_description/config/localization_params.yaml
```
# rviz
```bash
rviz2 \
  -d /home/deng/omove_ws/src/omove_description/config/localization_nav2.rviz \
  --ros-args -p use_sim_time:=true
```
# Nav2导航组件
```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  autostart:=true \
  use_composition:=False \
  params_file:=/home/deng/omove_ws/src/omove_description/config/nav2_params.yaml
```