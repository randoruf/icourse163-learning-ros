# 1. ROS通信架构

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
	
		


