# TODO

# Start

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make
```
```bash
cd ~/catkin_ws/src
git clone https://github.com/Zhao-Qihao/PerceptionSystem.git
```
```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```
```bash
source devel/setup.bash
roslaunch perception_launch perception.launch
```
