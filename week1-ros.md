# 1. 什么是ROS

目前越来越多的机器人、无人机甚至无人车都开始采用ROS作为开发平台。

以前，开发一个机器人需要花很大的功夫，你需要设计**机械**、画**电路板**、写**驱动程序**、**设计通信架构**、组装集成、调试、以及编写各种感知决策和控制算法，每一个任务都需要花费大量的时间。

如今的电机、底盘、激光雷达、摄像头、机械臂等等元器件都有不同厂家专门生产。社会分工加速了机器人行业的发展。而各个部件的集成就需要一个统一的**软件平台**，在机器人领域，这个平台就是机器人操作系统ROS。



## 1.1 ROS特点

- **分布式 - 点对点 (Node to Node)** 
	- ROS采用了**分布式的框架**，通过点对点的设计**让机器人的进程可以分别运行**，便于模块化的修改和定制，提高了系统的容错能力。
- 缺点
	- 通信实时性能有限
	- 系统稳定性尚不满足工业级要求

总体来说，ROS更适合科研和开源用户使用，



# 2. ROS setup 

## 2.1 Setup ROS 

Install ROS on windows (natively, instead of WSL).  Read [Installation/Windows - ROS Wiki](http://wiki.ros.org/Installation/Windows)

Install *Windows Terminal application*. Add a shortcut `ROS Noetic`  to Windows Terminal it should run `C:\opt\ros\noetic\x64\setup.bat`  if you launch a new terminal. 

<img src="image-20210206142340014.png" alt="image-20210206142340014" style="zoom: 50%;" />

Don't forget create a new file `go2ros.bat`

```
chdir /d C:\opt\ros\noetic\x64\
```

And add it to the `setting.json` in the *Windows Terminal* application, it will change the current directory to the ROS directory. 

But if you have your own project, modify the `C:\opt\ros\noetic\x64\` to the path of your own project. 



# 3. ROS Essential 

Some basic concepts in ROS 

## 3.1 Node and Topic 

- ***Node***
  - The minimum unit of execution to *process data/information*.
  - Similar to the concept of ***class*** in Object-Oriented Programming.   
  - Four types of node
  	- Publisher
  		- a node that *product contents* and then *send processed data* to other nodes via *topics*. 
  		- The publisher will update the topic at a specified frequency (for example, *10 Hz, 10 times per second*) . 
  		- For example, a small block that processes raw information from sensors like camera, encoders. 
  	- Subscriber
  		- a node that receives information via *topics*.
  		- Use a *"callback function"* to process received information (*the concept of "callback" is similar to JavaScript* or *interrupt service of CPUs)*.   
  			- Because callback function is an interrupt service, *it is only executed if new data is published*. 
  		- For example, the water level of a tank is lower than a specified value.  
  	- Service 
  	- Action 
- ***Topic***
	
	- a pipe between nodes for *information transport*. 
	
	- Similar to the `struct` in C language, it is basically a collection of different data types such as strings, integers, floating point numbers.  Each topic is like a variable, it has a *name* and a *type*. 
	
	- A topic can only have one value, but *multiple nodes* can publish to this topic.
	
		



## 3.2 Run Turtle Example on ROS

[理解简单海龟模拟程序 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/139373947)

Setup ROS Environment in a new terminal (but this is usually in *windows terminal*, so don't worry). 

```
C:\opt\ros\noetic\x64\setup.bat
```

Once you setup the ROS Environment, you can directly run ROS commands ( try to run `roscore` in the normal terminal) . 

![image-20210206145847799](image-20210206145847799.png)

launch ROS master (make sure that you run `setup.bat`). 

```
roscore
```

 Open another terminal, turtle simulation node 

```
rosrun turtlesim turtlesim_node
```

Open another terminal, turtle keyboard controller node 

```
rosrun turtlesim turtle_teleop_key
```

Try to press the arrow key on the `turtle_teleop_key` node. 



## 3.3 ROS Basic Command

To understand the relationship between nodes, open another terminal, run the command.

```
rqt_graph
```

(make sure that you have installed *graphviz* and `choco install graphviz` does the same thing as `apt-get`, remember add the path to the environment variables). 

You could also list all nodes 

```
rosnode list
```

(其中`rosout`是ros执行的, 用来存储其他节点的日志信息。)

To specify a node 

```
rosnode info <node_name>
```

<img src="image-20210206145436195.png" alt="image-20210206145436195" style="zoom: 50%;" />

You could also list all topics 

```
rostopic list
```

<img src="image-20210206161007127.png" alt="image-20210206161007127" style="zoom: 33%;" />

Topic data structure 

```
rostopic info <topic_name>
```

Print *topic contents* on the terminal (a *loop* and print at each second)

```
rostopic echo <topic_name>
```



## 3.4 Start a ROS Project 

微软的 Command Prompt 自带 `tree` utility 。

### 3.4.1 What is catkin

对于ROS这样大体量的平台来说，就采用的是CMake，并且ROS对CMake进行了扩展，于是便有了Catkin编译系统。

<img src="catkin.jpg" alt="img" style="zoom: 67%;" />



### 3.4.2 Initialize ROS Workspace 

catkin的工作空间，直观的形容就是一个仓库，里面装载着ROS的各种项目工程，便于系统组织管理调用。在可视化图形界面里是一个文件夹。我们自己写的ROS代码通常就放在工作空间中，本节就来介绍catkin工作空间的结构。

<img src="catkin_flow.jpg" alt="img" style="zoom:50%;" />

![image-20210206165140208](image-20210206165140208.png)

后两个路径由catkin系统自动生成、管理，我们日常的开发一般不会去涉及，而主要用到的是`src`文件夹，我们写的ROS程序、网上下载的ROS源代码包都存放在这里。

To get started, create a new folder (including the subfolder `src`) in arbitrary location.  

```
cd C:\Users\????\Desktop\
mkdir my_catkin_wp\src
```

Setup the correct ROS environment and initialize catkin workspace. 

```
C:\opt\ros\noetic\x64\setup.bat
catkin_make
```



### 3.4.4 ROS Package 



<img src="image-20210208202522468.png" alt="image-20210208202522468" style="zoom: 33%;" />

ROS package 是**最小的编译单位** (类似"器官"）。 

但最小的功能单位是**节点**， 类似于"细胞" 。ROS 的一个 Package 可以包含多个可执行文件/节点)。

同样要注意到 ROS package 的查找是递归的，所以以下文件结构也是允许的。

<img src="image-20210208202646623.png" alt="image-20210208202646623" style="zoom: 33%;" />



>  ***一个 Package 都包含什么呢？***

![image-20210208204802680](image-20210208204802680.png)

> Package 里面应该包含**源代码**。 

<img src="image-20210208203616719.png" alt="image-20210208203616719" style="zoom: 33%;" />

- `CMakeLists.txt`: 定义package的**依赖**、源文件、目标文件等编译规则，是package不可少的成分
- `package.xml`: 当前 package 自我描述， 例如包名、版本号、作者、依赖等信息，
- `src/`: 存放ROS的源代码，包括C++的源码和(.cpp)以及Python的module(.py)
- `include/`: 存放C++源码对应的头文件
- `scripts/`: 存放可执行脚本，例如shell脚本(.sh)、Python脚本(.py)



<img src="image-20210208222820117.png" alt="image-20210208222820117" style="zoom:50%;" />

<img src="image-20210208223102195.png" alt="image-20210208223102195" style="zoom:50%;" />



> Package 里面除了放代码文件， 还可以放**自定义通信格式**。

<img src="image-20210208204122589.png" alt="image-20210208204122589" style="zoom: 33%;" />

- `msg` : 消息
- `srv`  : 服务
- `action` : 动作

> Package 还能放入 Launch 和 Yaml 文件

<img src="image-20210208204651766.png" alt="image-20210208204651766" style="zoom: 33%;" />

- `launch` : 负责同时启动多个可执行文件
- `yaml` : 配置文件(configuration)



#### 3.4.4.1 常见的 ROS 包管理指令

查找某个 pkg 的地址 :  `rospack find <package_name>`

列出本地所有 pkg :  `rospack list`

跳转到某个 pkg 路径下(递归结构下节约时间) : `roscd <package_name>`

列举某个 pkg 下的文件信息(类似 `ls`) : `rosls <package_name>`

编辑 pkg中的文件(类似于 `vim`  ，其实就是没用的指令): `roscd <package_name> <fike_name>`

创建一个 pkg : `catkin_create_pkg <pkg_name> [deps]`

安装某个 pkg 所需的依赖 : `rosdep install [pkg_name]` 



#### 3.4.4.2 Write Your Own by Creating a New Package 

之前说过， ***ROS 最小的编译单位是 Package*** 。  如果要写一个程序， 就必须创建一个 ROS Package 。 

(很多 ROS 课程的每周作业都是要求提交一个 ROS Package) 。

如何创建一个新的 ROS Package 呢？ 首先进入 `src` 目录。

```
cd C:\...\<catkin_ws>\src
catkin_create_pkg <package_name> (<dep_name>)
```

如果你再写程序之前就知道自己要用到什么依赖， 可以把 `<dep_name>` 替换成其他 ROS Package 的名字（比如 很流行的 SLAM 算法 `gmapping`）， 当然大多数情况下你是不知道的。 



#### 3.4.4.3 Download Others' ROS Package 

如果你只是下载别人的包来玩玩 (不创建自己的 ROS Package). 

转到 catkin workspace 的 `src` 目录, 然后下载别人的 ROS Package (因为ROS是递归搜索的， 所以其实放在 `src` 下面的一些自建文件夹也可以) 

```
cd C:\...\<catkin_ws>\src
git clone https://github.com/DroidAITech/ROS-Academy-for-Begineers.git
```



#### 3.4.4.4 Install Dependencies 

刚下载下来的 ROS package 可能不能马上就编译能用， 很多时候会缺少各种依赖 （比如下载游戏来玩也会有 DirectX 的错误提示）。这时候你就要在 catkin workspace 里安装依赖了。 

Install a single specified ROS package 

```
cd C:\...\<catkin_ws>\
rosdep install <package_name>
```

Install all ROS package dependencies (ROS will recursively look at the `package.xml` of each package). 

```
cd C:\...\<catkin_ws>\
rosdep install --from-paths src --ignore-src --rosdistro=noetic -y
```

(`rosdistro=noetic` 要视乎当前 ROS 系统的版本， 我的写这篇笔记的时候是 `noetic`) 。



### 3.4.5 Compiler your ROS project 

Catkin 除了能够初始化工作空间， **最重要的作用是编译你写好的 ROS程序。**

写完代码都需要 Catkin 一下来编译代码。 

```
cd C:\...\<catkin_ws>\
catkin_make
```

最后一步是 source 一下 devel 中的 setup, 把这些包导入到 ROS 系统里。

```
C:\...\<catkin_ws>\devel\setup.bat
```

最后可以查一下是否已经把你的 Project 添加到 ROS 系统中

```
rospack list
```





## 4. Create Nodes in Python 

### Publisher Node 

### Subscriber Node 

