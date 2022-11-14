Ros Readme

Season only nodes/code go into main_robot (c++) or main_robot_py (python) packages
reusable nodes can create own package

TODO Reusable nodes:
- Camera publisher 
- IMU
- Color sensor
- Button
- Node Checker (reads config for nodes to publish status for)
- LED Robot Light Indicator (something like 10 lights to send on or off to)

common commands
rosdep install -i --from-path src --rosdistro galatic -y
colcon build
or
colcon build --packages-select src/network_table_publisher
. install/setup.bash
ros2 run network_table_publisher talker

docker-compose up

on pi to use only 1 cpu
export MAKEFLAGS="-j 1"

for ros2 bas install you might need to install
sudo apt install build-essential python3-colcon-common-extension -y

Yuh!

Officially switches to JAVA due to trobuleshooting errors. 2/22/22.
