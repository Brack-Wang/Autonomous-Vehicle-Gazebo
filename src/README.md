### Starter Code of Polaris GEM e2 Simulator for [ECE484](https://publish.illinois.edu/safe-autonomy/) and [CS588](http://luthuli.cs.uiuc.edu/~daf//courses/MAAV-22/588-2022-home.html)

#### University of Illinois at Urbana-Champaign

#### System: Ubuntu 20.04 + ROS Noetic (Gazebo 11)

#### Author: Hang Cui (hangcui1201@gmail.com)

This simulator was initially developed with ROS Melodic and Gazebo 9 in Ubuntu 18.04 for personal research in fall 2019. The Polaris GEM e2 vehicle was measured and modeled by Hang Cui and Jiaming Zhang using Solidworks. The compatible URDF files of simulator for RViz and Gazebo were constructed by Hang Cui. Later, this project was funded by the [Center of Autonomy](https://autonomy.illinois.edu/) at University of Illinois at Urbana-Champaign. It was further developed and merged into ROS Noetic and Gazeno 11 in the summer of 2021. This simulator is currently under development for research and teaching at University of Illinois at Urbana-Champaign.  

#### Polaris GEM e2 Vehicle

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









