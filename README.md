# Path Planning

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---

In this project the goal is to safely navigate around a virtual highway with other traffic that is driving up to 50 MPH. The project was implemented in C++, and the communication between the project and the Udacity simulator is done using WebSocket, where the simulator provides the car's localization, sensor fusion data and the sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible.

![]( https://github.com/shmulik-willinger/path_planning/blob/master/readme_img/path_planing.jpg?raw=true)


#### Limitations
The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another.

The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop.

The car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.


#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.


#### Point Paths
The project output a list of x and y global map coordinates. Each pair of x and y coordinates is a point, and all of the 30 points together form a trajectory.

Every 20 ms the car moves to the next point on the list. The car's new rotation becomes the line between the previous waypoint and the car's new location.


## Here is the data provided from the Simulator to the C++ Program:

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time.

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.


Cartesian (x,y) to Frenet (s,d)
![]( https://github.com/shmulik-willinger/path_planning/blob/master/readme_img/Frenet.jpg?raw=true)

Reference points for Lane changing
![]( https://github.com/shmulik-willinger/path_planning/blob/master/readme_img/lane_decision.jpg?raw=true)



## Implementation

The objectives of the project were achieved by the following steps:

#### 1. Construct Interpolated Waypoints of adjacent Area
The car uses a perfect controller and will visit every (x,y) point it receives in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. In this step the program builds a set of nearby waypoints, and uses 'getXY' and 'getFrenet' to produce a set of 0.5 meters waypoints for more accurate results.

#### 2. Proportional velocity control
The max speed limitation was set to 49.9 mph, in order to not exceed the road max speed of 50 mph. Whenever there is no vehicle ahead - the target velocity will use the default max speed. In case that our vehicle is close to another car - it will adjusts the target velocity to the speed of the car in front, with a safety distance ahead. The code can be found in main.cpp (from line 270)

#### 3. Lane decision
For the given lane and state, the program is trying to calculate if we need to change lane. The vehicle must be able to drive safety and avoid collisions by changing lanes only to available ones. The logic must pay attention to the a couple of criteria as 1. Other cars ahead, 2. cars in nearby lanes, 3. safety (cars location and speed).
The output of this step is whether to change lane and when. The code can be found in Vehicle.cpp (from line 17)

#### 4. Trajectory generation
Using a corresponding Frenet coordinates (s,d) we need to create (x,y) coordinates of waypoints ahead of the vehicle. The car should progress to each on of the points in every 0.02 seconds, so we're generating evenly smooth trajectory by using the spline function over the waypoints. The code can be found in main.cpp (from line 354)

#### Previous path
There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, so the program will store the last points we have used in order to have a smooth transition. previous_path_x, and previous_path_y are helpful for this transition since they show the last points given to the simulator controller with the processed points already removed.


Change lane              |  Stay in lane
:---------------------:|:---------------------:
![]( https://github.com/shmulik-willinger/path_planning/blob/master/readme_img/change_lane.jpg?raw=true)  |  ![]( https://github.com/shmulik-willinger/path_planning/blob/master/readme_img/keep_lane.jpg?raw=true)

Driving 10 miles without collisions  |  Driving at max speed
:---------------------:|:---------------------:
![](https://github.com/shmulik-willinger/path_planning/blob/master/readme_img/10_miles.jpg?raw=true)  |  ![]( https://github.com/shmulik-willinger/path_planning/blob/master/readme_img/max_speed.jpg?raw=true)


Dependencies and Executing
---

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
  * The Simulator which contains the Path Planning Project can be downloaded from [here](https://github.com/udacity/self-driving-car-sim/releases).

#### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.



Process results
---
The vehicle successfully drive a lap around the track and according to the speed limit. Max Acceleration and Jerk were not Exceeded. The car does not have collisions, and stays in its lane (except for the time between changing lanes). The program performs optimized lane changing, this means that the car only changes into a lane that improves its forward progress.

The following video demonstrate a smoothly lane changing when passing another car in the road:

![]( https://github.com/shmulik-willinger/path_planning/blob/master/readme_img/change_lanes.gif?raw=true)
