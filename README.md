# Gazebo Simulator 
---
# Installation
## To succeed run the simulator, you should:
1. mkdir Simulator
2. cd Simulator
3. mkdir ./Store
4. git clone https://github.com/Brack-Wang/Autonomous-Vehicle-Gazebo-CS588.git
5. Change name of folder "Autonomous-Vehicle-Gazebo" to "src"
6. Install yolov3.weight for object detection
	- install yolov3.weight from https://pjreddie.com/media/files/yolov3.weights
	- Put the weight file in ./Store
7. Download Rosbag to publish essential real topics:
Lane Detect: https://drive.google.com/file/d/1uh7e4kP_bxRsgM3ZGiiM5_wImATIF3oE/view?usp=share_link
Object Detect: https://drive.google.com/file/d/1jU4KOYWxC1Cf9K1SX1XIh1JPtUfYRkm8/view?usp=share_link
Use rosbag to run it when using camera_det.py
---
# Simulator Command cheat sheet
1. **Compile**
```
source devel/setup.bash  
catkin build 
# or you could use catkin_make to compile
```
2. **Open Gazebo**
```
# 4 gazebo wold

1) World which has pedestrain model for object detection
source devel/setup.bash  
roslaunch gem_launch gem_init.launch world_name:="mp0.world"  

2) World with waypoints which could use pure pursuit/stanley_sim directly
source devel/setup.bash  
roslaunch gem_launch gem_init.launch world_name:="track1.world" 

3) World without waypoints
source devel/setup.bash  
roslaunch gem_launch gem_init.launch world_name:="track2.world" 

4) World for real Car
source devel/setup.bash  
roslaunch gem_launch gem_init.launch world_name:="highbay_track.world" x:=-1.5 y:=-21 yaw:=3.1416  
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
6. **Camera Detecion**
```
source devel/setup.bash 
rosrun camera_vision camera_det.py

# Show topics information
source devel/setup.bash 
rostopic echo /front_single_camera/object_detection

```
---
# Keys
## Camera Detection
1. Message
which detect pedestrain and lanes, return bounding box information of detected objects and the middle line of lanes
returns a ros msgs named as "Detected_info" 
```from camera_vision.msg import Detected_info```; 
```
DetectBox_msg: 

float32[] object_distance	 #distance of object from car
float32[] object_x		# Horizontal coordinate of object
float32[] object_y	     # verticle coordinate of object
float32[] classId 	 # Id of person is "0"
float32[] confidence # confidence of detecting an object
float32[] middle_lane # positions of points in middle lines [x1, y1, distance1, x2, y2, distance2, x3, y3, distance3...]
```

2. Topic
```/object_detection```

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
### Track1 Environment

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

