


## Overview
This package compares two poses of type geometry_msgs/PoseWithCovarianceStamped and geometry_msgs/PoseStamped
using the TimeSync policy from message filters. Basically the callback is only executed when message of approximate
same timestamp arrive which allows to compare poses more accurately.

If poses of another type have to compared, the callback and the type of the message filter subscriber has to be changed.
the way to run the node is to run the command 
rosrun tf_pose_comparator compare_node

This will subscribe two poses of the type and name "/ground_truth_pose", geometry_msgs::PoseStamped and
"/amcl_pose", geometry_msgs::PoseWithCovarianceStamped respectively. The topic names can be remapped in 
a launch file if that is the only modification required.
## Installation

### Installing dependencies
    
Or better, use `rosdep`:

	sudo rosdep install --from-paths src

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),

	sudo rosdep install --from-paths src

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/nitishk162/tf_pose_comparator.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make


### Running in Docker

Docker is a great way to run an application with all dependencies and libraries bundles together. 
Make sure to [install Docker](https://docs.docker.com/get-docker/) first. 

First, spin up a simple container:

	docker run -ti --rm --name ros-container ros:melodic bash
	
This downloads the `ros:melodic` image from the Docker Hub, indicates that it requires an interactive terminal (`-t, -i`), gives it a name (`--name`), removes it after you exit the container (`--rm`) and runs a command (`bash`).

Now, create a catkin workspace, clone the package, build it, done!

	apt-get update && apt-get install -y git
	mkdir -p /ws/src && cd /ws/src
	git clone https://github.com/nitishk162/tf_pose_comparator.git

	cd ..
	rosdep install --from-path src
	catkin_make
	source devel/setup.bash
	rosrun tf_pose_comparator compare_node

This will run the node and subscribe to the topics being compared and results will be logged into the logs/delta_error_file.dat. This file gets appended on every run.
 "/ground_truth_pose", geometry_msgs::PoseStamped - This is the ground pose against which the next pose is compared against
"/amcl_pose"- geometry_msgs::PoseWithCovarianceStamped