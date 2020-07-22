Maze-Solving-Robot
Roles: Algorithm & Frontend Developer

In this project, an autonomous robot was developed to: 
1) Traverse and map an unknown arena, avoiding obstacles 
2) Plan and follow fastest path between two points 
3) detect, identify and locate images on obstacles

Four subsystems were involved: 
1) Arduino(robot), which is in charge of assembly and control of the physical robot
2) Raspberry Pi, which in in charge of inter-subsystem communications and image recognition model
3) Android tablet simulator, which consists of a user interface for real-time monitoring of robot position, map exploration and image detection status. 
4) Algorithm and desktop simulator
   4.1) Algorithm, which functions as the brain of the robot during exploration, fastest path and image detection.
   4.2) A desktop simulator that is used to:
          4.2.1) simulate algorithms and expected robot behaviour in standalone mode 
          4.2.2) same functionality as the Android tablet simulator in integrated mode.

As I was in charge of 4.Algorithm and desktop simulator, only sourcecode for this subsystem is included in the repo.

Highlights:
* Developed and implemented an efficient maze-solving algorithm with the ability to:
   - map and traverse the arena, 
   - capture and locate images with minimal time and turning costs.
* Implemented A* algorithm for the shortest path through a pre-defined waypoint
* Designed and establised communications between Algorithm and Raspberry Pi over WIFI
* Built a desktop simulator app that:
   - Similar to the Android tablet simulator, it displays robot position&facing, map exploration and image detection status in real time in integrated mode
   - Simulates algorithms and expected robot behaviour in standalone mode, assuming robot sensors have 100% accuracy in detecting its surroundings
   - Video demo of the desktop simulator (in standalong mode):
![ezgif com-video-to-gif](https://user-images.githubusercontent.com/44166629/88133335-5f63a700-cc14-11ea-825a-6dc47fd5d0dd.gif)
