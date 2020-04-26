# Mechatronics Project

## Launching the simulation
The ```TURTLEBOT3_MODEL``` environment variable need to be exported globally :
```bash
cd FinalProject; export TURTLEBOT3_MODEL=burger
```
Then, in separate terminals, enter the following :

### Launching the environment
```bash
roslaunch ttb3_custom.launch
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
python robot.py
```
