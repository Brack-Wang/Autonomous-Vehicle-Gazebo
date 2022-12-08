# HW 2: Waypoint Following PID

In this HW, you need to implement PID controller to follow waypoints along a path.

## Code structure

```
.
+-- gem_waypoint_pid
|  +-- waypoints
|  |  +-- wps.csv              # waypoints list
|  |  +-- records_wps          # visualization of waypoints
|  +-- scripts
|  |  +-- gen_waypoint.py      # for generating waypints
|  |  +-- pid.py               # for implementing PID controller
|  |  +-- follow_waypoints.py  # main script to interact with the simulator
```

## To be implemented

All code snippets to be implemented by yourself has been marked with `# TODO: ...`

### 1. PID controller

Please implement PID controller in `gem_waypoint_pid/scripts/pid.py`.

### 2. Application of PID controller

Please complete the following parts in `gem_waypoint_pid/scripts/follow_waypoints.py`:

- transforming the goal point into the vehicle coordinate frame in function `start_drive`
- define your feedback value for PID control in function `start_drive`
- Set your own sets of weights for P, I, D terms in `__init__`.

## How to run

Assume your ROS workspace locates at `$WS_ROOT`. Please place folder `gem_waypoint_pid` under `$WS_ROOT/src/POLARIS_GEM_e2/polaris_gem_drivers_sim`.

Then `rosrun gem_waypoint_pid follow_waypoints.py`. You should see vehicle move in the middle of the road in the simulator.

## What to be submitted

Please submit the following:

1. Your completed `gem_waypoint_pid`
2. Two videos showing the running of your PID contollers with **two sets of P, I, D terms**.
