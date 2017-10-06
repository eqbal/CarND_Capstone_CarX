# Self-Driving Car Nanodegree Capstone Project (CAR-X Team)

### Overview

The System Integration project is the final project of the Udacity Self-Driving Car Engineer Nanodegree.

The goal of this project is to code a real self-driving car to drive itself on a test track using [ROS](http://www.ros.org/) and [Autoware](https://github.com/CPFL/Autoware). The project is coded to run on Udacity simulator as well as on Udacity's own self-driving car [CARLA](https://medium.com/udacity/how-the-udacity-self-driving-car-works-575365270a40).

![CARLA](./imgs/udacity-carla.jpg)

As a team, we built ROS nodes to implement core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint following.

For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Architecture

Using the Robot Operating System (ROS), each team member has developed and maintained a core component of the infrastructure that is demanded by the autonomous vehicle. The three core components of any good robot are the following:

- Perception: Sensing the environment to perceive obstacles, traffic hazards as well as traffic lights and road signs.

- Planning: Route planning to a given goal state using data from localization, perception and environment maps.

- Control: Actualising trajectories formed as part of planning, in order actuate the vehicle, through steering, throttle and brake commands.

![ROS Architecture](./imgs/ros-architecture.png)

### ROS Nodes Description


- Waypoint Updater node (**waypoint_updater**)

	This node publishes the next **200** waypoints that are closest to vehicle's current location and are ahead of the vehicle. This node also considers obstacles and traffic lights to set the velocity for each waypoint.

	This node subscribes to following topics:

	- **base_waypoints**: Waypoints for the whole track are published to this topic. This publication is a one-time only operation. The waypoint updater node receives these waypoints, stores them for later use and uses these points to extract the next 200 points ahead of the vehicle.

	- **traffic_waypoint**: To receive the index of the waypoint in the base_waypoints list, which is closest to the red traffic light so that the vehicle can be stopped. The waypoint updater node uses this index to calculate the distance from the vehicle to the traffic light if the traffic light is red and the car needs to be stopped.

	- **current_pose**: To receive current position of vehicle.


	The functionality of the `waypoint updater` node is to process the track waypoints that are provided from the `waypoint_loader` and provide the next waypoints that the car will follow. The speed is adjusted in the presence of a red traffic light ahead.

	For the described operation the following steps are followed, provided that the track waypoints have already been loaded:

	- __Identify the car's position in the car__ (`pose_cb`) :
	Knowing the car's position in (x,y) coordinates, the closent track point is returned as an index ranked by its Euclideian distance.  Then the next few points ahead (defined by LOOKAHEAD_WPS constant) will be the final uprocessed waypoints.

	-  __Processing of the waypoints__ (`waypoints_process`):
	The fnctions loops throught the subsequent waypoints and the following options can take plance.
		- __Traffic light not close or green:__ The waypoints velocity is upated with the maximum allowed one
		- __Traffic light red and close__: The car is required to stop. The velocity is set to 0
		- __Traffic light red within deceleration distance__: Car is approaching the traddice light but is not so close yet. Velocity is linearly dropping.

	After the waypoints are updated they are published and are send through the waypoint follower to the `twist controller` which is implementing the actuator commands.
	This node publishes to following topics:

	- **final_waypoints**: Selected 200 waypoints including their velocity information are published to this topic.


- Twist Controller Node **(dbw_node)**

	This node is responsible for vehicle control (acceleration, steering, brake).

	This node subscribes to the following topics:

	- **dbw_enabled**: Indicates if the car is under dbw or driver control.
	- **current_velocity**: To receive the current velocity of the vehicle.
	- **twist_cmd**: Target vehicle linear and angular velocities in the form of twist commands are published to this topic.

	This node publishes to following topics:

	- **steering_cmd**: Steering commands are published to this topic.
	- **throttle_cmd**: Throttle commands are published to this topic.
	- **brake_cmd**: Brake commands are published to this topic.

	To calculate vehicle control commands for steering, throttle and brake this node makes use of Controller (as coded in twist_controller.py).
	
	The throttle of the car is calculated based on the current velocity and the target velocity and controlled by a PID controller for error correction. The PID controller uses the following parameters.

	```
	 kp   = 0.3
	 ki   = 0.003
	 kd   = 4.0
	```
	The parameters may need to be tweaked in real world situation as the current settings were for the simulator.

- The `Yaw Controller` controls the steering angle based on the current linear velocity and the target linear and angular velocity.

- The brake value is based on multiple parametrs, viz. the mass of the vehicle, current velocity of the car and the radius of the wheel. The deceleration is limited by the parameter 'decel_limit'.  Brake is applied only if the target velocity is less than the current velocity. The brake value is in N/m and the formulae used for calculting the brake is as follows.

	```
	longitudinal_force = mass_of_car * acceleration (or deceleration)
	    
	Torque needed to stop/ accelerate = longitudinal_force * wheel_radius
	    
	```
 
	The torque is supplied as the brake value in N/m limited by the decel_limit parameter.
    
- Traffic light detection node **(tl_detector)** ... to be continued

- SVM Training Node for Traffic Light Detection and Classification **(tl_ detecor_train)** ... to be continued
	- For Simulator (SVM)... to be continued
	- For Real World (FCN) ... to be continued with much details of the network implementation

- TLClassifier class **(tl_classifier)** ... to be continued

### Visualization module

To facilitate debugging of the system a visualization module was created for RVIZ. Information about waypoints, car location, upcoming stop light and also traffic light and their status - red, yellow or green. The screenshot below is an example of RVIZ debugging session:

![visualization](./imgs/visualization_module.png)

### Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Usage

1. Clone the project repository

	```bash
	git clone https://github.com/udacity/CarND-Capstone.git
	```

2. Install python dependencies

	```bash
	cd CarND-Capstone
	pip install -r requirements.txt
	```

3. Make and run styx

	```bash
	cd ros
	catkin_make
	source devel/setup.sh
	roslaunch launch/styx.launch
	```

4. Run the simulator

### Running code on CARLA

In order to run the code on CARLA, it is necessary to use a different classifier. At the moment we are using an SVM in the simulator environment, while a FCN is used to detect traffic lights in the real world.

Therefore, it is necessary to download the trained FCN network (based on VGG) snapshot.

Due to the size of the file, it cannot be hosted in GitHub, so please use the following link to download: [Trained FCN snapshot](some_url_after_checking_with_carx_team).


### Collaboration

Each of the team members in `Car-X` team should handle a core component.

Assuming `Eqbal` is handling the `dbw_node`:

- Create a branch with the first two name initiative followed by the node name: `git checkout -b eq-dbw_node`

- Commit the changes: `git commit -am "descriptive but short commit"`

- Once you are stuck push your branch so everyone in the team can check out the updates: `git push -u origin eq-dbw_node`

- For testing please keep your python notebooks inside `./notebooks` folder.

- Don't apply the changes directly to `master`

### Task break down table
To break down tasks and manage our task we'll be using [Github Project management](https://github.com/marketplace/category/project-management) check out the link below:

https://github.com/eqbal/CarND_Capstone_CarX/projects/1

### Team Members:

  - [Eqbal Quran](www.eqbalq.com) (info@eqbalq.com)

	> Eki has already written a book about Ruby 5.0. It’s currently
sealed up. In three years, Matz is going to open the book to see if the
language design team got it right.

  - Jaime Blasco (pinoch0@gmail.com)
	> Jaime's addition operator doesn't commute; it teleports to
where he needs it to be

  - Dimitrios Mavridis (dmavridis@gmail.com)

	> Dimitrios's code doesn't follow a coding convention. It is the coding convention.

  - Mani Srinivasan (srnimani@gmail.com)
	> Mani does not use revision control software. None of his code has ever needed revision.

  - Volodymyr Seliuchenko (volodymyr.seliuchenko@gmail.com)
	> Volodymyr's coding reputation is only as modest as it is because of integer overflow (SQL Server does not have a datatype large enough)

### Real world testing

1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```

