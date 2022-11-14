# Gazebo Simulator of CS 588
---
# Installation
## To succeed run the simulator, you should:
1. mkdir Simulator
2. cd Simulator
3. mkdir ./Store
4. git clone https://github.com/Brack-Wang/Autonomous-Vehicle-Gazebo-CS588.git
5. Change name of folder "Autonomous-Vehicle-Gazebo-CS588" to "src"
6. Install yolov3.weight for object detection
	- install yolov3.weight from https://pjreddie.com/media/files/yolov3.weights
	- Put the weight file in ./Store

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

# 4 gazebo wold
1) World which has pedestrain model for object detection
roslaunch gem_launch gem_init.launch world_name:="mp0.world"  

2) World with waypoints which could use pure pursuit/stanley_sim directly
roslaunch gem_launch gem_init.launch world_name:="track1.world" 

3) World without waypoints
roslaunch gem_launch gem_init.launch world_name:="track2.world" 

4) World for real Car
roslaunch gem_launch gem_init.launch world_name:="highbay_track.world"  
```

3. **Show odometers on Rviz**
```
source devel/setup.bash  
roslaunch gem_launch gem_sensor_info.launch 
```
4. **Move with pure pursuit for track1.world**
```
source devel/setup.bash  
rosrun gem_pure_pursuit_sim pure_pursuit_sim.py  
```
5. **Move with stanley_sim for track1.world**
```
source devel/setup.bash  
rosrun gem_stanley_sim stanley_sim.py  
```
6. **Pedestrin Detecion**
```
source devel/setup.bash 
rosrun camera_vision camera_det.py
```
---
# Keys
## Pedestrian Detection
returns a ros msgs named as "Boudingbox" 
```from camera_vision.msg import Boudingbox```; 
```
Boundingbox_msg: 

float32[] center_x	 #center of Bouding box
float32[] center_y
float32[] width	     # width of Bounding box
float32[] height 	 # height of Bouding box
float32[] distance	 # distance to object
float32[] confidence # confidence of detecting an object
int32[] classId	     # clssId = 0 means pedestrain
```

If you don't want to see visulization, decomment the usage of ```draw_prediction``` in utils.py

---

# Target
#### FUNCTION

- (BASIC 1) 沿着车道行驶

- (BASIC 2) 检测STOP SIGN/车道上的静止物体，刹车，等消失后继续行驶

- (ADVANCE) 检测到车道前方的静止物体，绕道


#### PERCEPTION
- 车道线检测（转弯）

	- INPUT: Camera 

	- OUTPUT: road_path to MOTION PLANNING 

- 物体检测（行人， STOP SIGN）

	- INPUT: Camera/Lidar

	- OUTPUT: 4 world coordinates of obstacle  [Rectangular]  / 8 world coordinates of obstacle  [Cube]
		  

#### MOTION PLANNING
- 车道保持

	- INPUT: road_path

	- OUTPUT: trajectory to CONTROL

- 识别STOP SIGN/车道上静止物体并刹车

	- INPUT:  coordinates of obstacle

	- OUTPUT: trajectory to CONTROL
	
#### CONTROL 

- MPC



# Others
## Useful Link
- Original Code of simulator

https://github.com/hangcui1201/POLARIS_GEM_e2_Simulator

- Original Code of  Real Car

https://github.com/hangcui1201/POLARIS_GEM_e2_Real

- Handbook

http://www.autolabor.com.cn/book/ROSTutorials/

## Cammands
#### Track1 Environment

$ source devel/setup.bash  
$ roslaunch gem_launch gem_init.launch world_name:="track1.world"  

$ source devel/setup.bash  
$ roslaunch gem_launch gem_sensor_info.launch  


##### Demo of Pure Pursuit Controller

$ source devel/setup.bash  
$ rosrun gem_pure_pursuit_sim pure_pursuit_sim.py  


##### Demo of Stanley Controller

$ source devel/setup.bash  
$ rosrun gem_stanley_sim stanley_sim.py  

### Track2 Environment

$ source devel/setup.bash  
$ roslaunch gem_launch gem_init.launch world_name:="track2.world" y:=-98.5  

### Example Environment

$ source devel/setup.bash  
$ roslaunch gem_launch gem_init.launch  

### Highbay Environment

$ source devel/setup.bash  
$ roslaunch gem_launch gem_init.launch world_name:="highbay_track.world" x:=-1.5 y:=-21 yaw:=3.1416  

