# ROS常用工具



## Gazebo

机器人仿真工具，类似的还有 ***V-REP***。 

但 Gazebo 对 ROS 支持更好（毕竟是亲爹出的）



## Rviz

方便可视化数据。（Gazebo 是仿真，提供虚拟场景。注意 Rviz 的插件都是 Subscriber ， 并不负责提供虚拟场景， 主要负责**感知任务**）。

可以在里面添加机器人模型，然后选择好 fixed frame 为**机器人的坐标**即可显示机器人。

如果此时继续添加雷达数据， 就可以获得类似 LiDAR 的效果图。

除了这个， 还可以结果 RGB 和 Depth 信息获得 Point Cloud 图（有颜色和 x,y,z信息）。

SLAM 里还会用到 map 和 path 呢。

还可以看一下 Collision 时用的简化模型 （点 collision enable）。



## Rqt

基于 Qt 开发的工具。

- `rqt_graph`  显示通信节点、主题等
- `rqt_plot` 绘制曲线（机器人控制常用， 比如 PID调试）
- `rqt_console` 查看日志



## rosbag

记录和回访数据流。（例如记录无人车的上路数据）

原理是创建一个 node ， 然后记录被选择的 topics 

记录某些 topic 到 bag 中

```
rosbag record topic1_name topic2_name
```

记录所有 topics 到 bag 中

```
rosbag record -a
```

回放 bag (记得先开启 `roscore` 的 master)

```
rosbag play <bag-files>
```



## Moveit!

[moveit! · 中国大学MOOC———《机器人操作系统入门》讲义 (gitbooks.io)](https://sychaichangkun.gitbooks.io/ros-tutorial-icourse163/content/chapter5/5.6.html)

它融合了研究者在运动规划、操纵、3D感知、运动学、控制和导航方面的**最新进展**，为操作者提供了一个易于使用的平台，使用它可以开发先进的机器人应用，也被广泛应用于工业，商业，研发和其他领域。由于以上特性，moveit！一跃成为在机器人上最广泛使用的开源操作软件，**截止2017年，已经被用于超过65台机器人**。