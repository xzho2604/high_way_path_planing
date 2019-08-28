# CarND-Path-Planning-Project
## Model Documentation
![alt text](demo.gif)
### Rubit Poins:
![alt text](42miles.png)

#### The car is able to drive at least 4.32 miles without incident..
The car can go the best record of 42 miles withou any inccidents as indicated in the pic above.

#### The car drives according to the speed limit.
The car never exceed the speed limit during its drive

#### Max Acceleration and Jerk are not Exceeded.
No Max Acceleration and Jerk warning during the drive

#### Car does not have collisions.
Car does not collide with other cars on the road

#### The car stays in its lane, except for the time between changing lanes.
Car would stays in the middle of the lane unless lane changing

#### The car is able to change lanes
Car will change lane when the current traffic in the current lane is too slow


### Path Generation:
#### Minimise Jerk
There are several to ensure that path genrated would be smooth
* Use the past planning point for current path planning:<br/>
I have include the past planning path that has not been excuted into the current path planing,kind of continue the path planing from the previous one , inducing the car yaw and volicity from the previous path as well as current car status so that the car trasition into this path planning would be smooth.

* Using the spline libarary to generate very smooth path:<br/>
To ensure the cure that spline libaray generate is smooth as possible, a spares spaced point has been choosen instead of all the points on the planning path. in this way the curve generated is much smoother.

* The speed change difference at each unit of time:<br/>
This parametre has been tuned so that the car's acceleration is niether too slow or too fast. To enable car to brake in time as well accelerate without jerk. With some epxperiments it turns out that 2.5 is the best option in line 118.

#### Collision Avoidance
Couple of strategies have been used to avoid collision with other vehicles most of them are applied to lane change stafty logic. There are some constant being defined as folow :
```c++
         // define some constants
         const double BUF_DIS = 30; // buffer distance to keep with car in the front
         const double MAX_SPEED = 49.5; // speed limit
         const double SPEED_DEL = 0.25; // speed increase each time
         const double LANE_WIDTH = 4; // speed increase each time
         const double NUM_LANE = 3; // speed increase each time
 
```

* Keep Lane: <br/>
Car will keep in the centre of the lane and drive at max speed allowed in normal cases. If there is car infront of our car and 
we can not change to either left nor right lane then we keep staying in the lane with a safty margin of distance with the car infront of us. if we gets too close ,then we reduce the speed, if there is no car within the safty margin distantance and we are below the speed limit then we increase the speed. we define staty margine here as 30 metres

* Change Left/Right: <br/>
When there is car infront of us within the saftey distance ,and is below the speed limit , we need to consider changin lanes.
we will check all other cars that are in other lanes, if there is a distance of gap of both 30 metres in the front and back that at the end of the current path planing(other cars are estimated position at the same of the current car at the end of the path planning), then we could consider change lane. But here the planining result is based on the end of the previous path planing which is in the future, so we need to check as well right now if the condition is okay to change lane, as describled in line 176 -179 as well as 188-191.

* Switch to middle lane:<br/>
When possible(condition allowed), we would prefer to stay in the middle lane ,since we have more options in the middle lane than in the other two lanes.

* Middle lane switch to side lanes:<br/>
When we are in the middle lane , and there is option to swith to both lanes(both lane is okay to switch to as the logic describled previously), I have computed an average speed in each lane at the sensor fusion stage , in this situation we would prefer the lane with average speed that is faster or the lane with no car.(line 208 - 214)


## 
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

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

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

