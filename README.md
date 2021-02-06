<https://www.edx.org/course/hello-real-world-with-ros-robot-operating-system>

![](2020-12-31-11-57-38.png)



- **Week 1: ROS Essentials.** Introduction to ROS Topics, Services, Actions and Nodes. Simple interaction with the course simulation environment.
- **Week 2: Build your own robot environment.** Software representation of a Robot using Unified Robot Description Format (URDF), ROS parameter server and adding real-world object representations to the simulation environment.
- **Week 3: Autonomous Navigation.** Map creation with GMapping package, autonomously navigate a known map with ROS navigation.
- **Week 4: Manipulation.** Motion planning, pick and place behaviors using industrial robots with ROS MoveIt!
- **Week 5: Robot Vision.** Object detection, pose estimation.
- **Week 6: Final Project.** ROS file system, basic concepts of behavior design with state machines, build a production line application with two industrial robot arms and a mobile robot.

---

# Week 0 - ROS setup 

Install ROS on windows (natively, instead of WSL). 

[理解简单海龟模拟程序 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/139373947)

After installing, launch ROS master. 

```
roscore
```

 Open another terminal, turtle simulation node 

```
rosrun turtlesim turtlesim_node
```

Turtle keyboard controller node 

```
rosrun turtlesim turtle_teleop_key
```

