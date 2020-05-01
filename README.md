# Mechatronics Project

## Launching the simulation
The ```TURTLEBOT3_MODEL``` environment variable needs to be exported globally :
```bash
cd MechatronicsProject; export TURTLEBOT3_MODEL=waffle
```
Then, in separate terminals, enter the following :

### Launching the environment
```bash
roslaunch ttb3_custom.launch map:=$(pwd)/maps/maze4.xml
```
### Getting rviz to work :
```bash
rosrun tf static_transform_publisher 0 0 0 0 0 0 odom base_scan 50
```
```bash
rosrun tf static_transform_publisher 0 0 0 0 0 0 odom camera_rgb_optical_frame 50
```
```bash
rviz -d pblart.rviz
```

### Launching autonomous control :
```bash
python scripts/robot2.py
```
### Target detection using camera
```bash
python scripts/cam.py
```
