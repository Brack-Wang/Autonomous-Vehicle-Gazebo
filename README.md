# Gazebo Simulator of CS 588
---

# Simulator Command cheat sheet
1. **Compile**
```
source devel/setup.bash  
catkin build (catkin_make)
```
2. **Open Gazebo**
```
source devel/setup.bash  
roslaunch gem_launch gem_init.launch world_name:="track1.world"  
```
3. **Show odometers on Rviz**
```
source devel/setup.bash  
roslaunch gem_launch gem_sensor_info.launch 
```
4. **Move with pure pursuit**
```
source devel/setup.bash  
rosrun gem_pure_pursuit_sim pure_pursuit_sim.py  
```
5. **Move with stanley_sim**
```
source devel/setup.bash  
rosrun gem_stanley_sim stanley_sim.py  
```
6. **Camera Vision**
```
source devel/setup.bash 
rosrun camera_vision camera_vision.py
```

---
# Installation
## To succeed run the simulator, you should:
1. **Install yolov3.weight for object detection**
- install yolov3.weight from https://pjreddie.com/media/files/yolov3.weights
- Put the weight file in ./src/gem_vision/camera_vision/scripts/Detector/

---

# Target
## FUNCTION

- (BASIC 1) 沿着车道行驶

- (BASIC 2) 检测STOP SIGN/车道上的静止物体，刹车，等消失后继续行驶

- (ADVANCE) 检测到车道前方的静止物体，绕道


## PERCEPTION
- 车道线检测（转弯）

	- INPUT: Camera 

	- OUTPUT: road_path to MOTION PLANNING 

- 物体检测（行人， STOP SIGN）

	- INPUT: Camera/Lidar

	- OUTPUT: 4 world coordinates of obstacle  [Rectangular]  / 8 world coordinates of obstacle  [Cube]
		  

## MOTION PLANNING
- 车道保持

	- INPUT: road_path

	- OUTPUT: trajectory to CONTROL

- 识别STOP SIGN/车道上静止物体并刹车

	- INPUT:  coordinates of obstacle

	- OUTPUT: trajectory to CONTROL
	
## CONTROL 

-MPC
