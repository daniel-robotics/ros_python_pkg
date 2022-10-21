# ros_python_pkg
This repo is a complete Catkin package. Create a new repo based on this template, then clone it to the **src/** directory of an existing Catkin workspace.
For an easy-start Catkin workspace, see here: https://github.com/daniel-robotics/ros_catkin_ws .


## Structuring a Catkin Package for ROS and Python
The structure of a Catkin package varies depending on its intended use, but this one is designed for *simplicity* and *flexibility*, requiring minimal configuration and providing examples and templates to quickly get new ROS nodes up and running.

```
ros_python_pkg/     # Root folder defines the name of a Catkin package
|   config/                 # Config foler holds .yaml files, which hold parameters for the ROS parameter server (rosparam).
|   |
|   |   example_nodes.yaml      # Parameters used by the two example nodes.
|   |
|   launch/                 # Launch folder holds .launch files, each of which starts one or more ROS node.
|   |
|   |   example_nodes.launch    # Launches example_publisher and example_subscriber by importing generic_node.launch
|   |   generic_node.launch     # Launches a node using a variety of configuration options
|   |
|   nodes/                  # Nodes folder holds executable python files which "bootstrap" a ROS node.
|   |
|   |   example_publisher       # Python script (no .py file extension) which instanciates a ROS node (ExamplePublisher).
|   |   example_subscriber      # Python script (no .py file extension) which instanciates a ROS node (ExampleSubscriber).
|   |
|   scripts/                # Scripts folder holds executable scripts for non-ROS functionality.
|   |
|   |   template_py_script      # Python script (no .py file extension) which does NOT start a ROS node (but still depends on "src")
|   |   template_sh_script      # Bash script (no .sh file extension)
|   |
|   src/                    # Src folder holds all python source code
|   |
|   |   ros_python_pkg/         # This Python package should have the same name as the Catkin package (ros_python_pkg in this case)
|   |   |   
|   |   |   util/                   # A python sub-package
|   |   |   |   
|   |   |   |   __init__.py             # Designates this folder as a package. Just imports everything from this folder.
|   |   |   |   config_utils.py         # Common utilities for configuring rospy nodes
|   |   |   |
|   |   |   examples/               # A python sub-package
|   |   |   |
|   |   |   |   __init__.py             # Designates this folder as a package. Just imports everything from this folder.
|   |   |   |   example_publisher.py    # Example class extending BaseNode which implements a simple publisher.
|   |   |   |   example_subscriber.py   # Example class extending BaseNode which implements a simple subscriber.
|   |   |   |   
|   |   |   __init__.py         # Designates this folder as a package. Just imports everything from this folder.
|   |   |   base_node.py        # Abstract class which provides basic node functionality.
|   |   |   
|   |
|   CMakeLists.txt          # Catkin-required file which defines the build process  
|   install.sh              # Installs dependencies for this package (edit as required)
|   package.xml             # Catkin-required file which defines the package (name, dependencies on other packages, etc)  
|   requirements.txt        # List of python packages required to be installed by Pip
|   setup.py                # Installs nodes, scripts, launch files, and Python packages (src) such that other packages (and ROS) can find them
|
```

## Initial Setup

1. Create new Github repo as a copy of this template, and clone to your Catkin workspace's **src/** folder.
2. Rename "your_pkg/src/ros_python_pkg/"  to  "your_pkg/src/your_pkg/".
3. Configuration: Need to edit the following files:
    - package.xml      : Edit package name, description, author, etc. Add <depend> tags as necessary to link to other Catkin packages.
    - CMakeLists.txt   : Edit package name. Need to add package dependencies here as well; same ones as in package.xml.
    - install.sh       : Edit if this package has any special installation procedures; delete if not.
    - requirements.txt : Edit if this package requires any Pip packages which are not available as standard ROS dependencies; delete if not.
 
At this point the package should build successfully if all dependencies are met.

## Launching the Example Nodes

Build the catkin workspace. From the root of the workspace, run:
```
roslaunch your_pkg example_nodes.launch
```
Two nodes are launched:
 - Publisher (src/ros_python_pkg/examples/example_publisher.py):
        Publishes values at the rate set by the "rate" parameter to the "example_topic" topic.
 - Subscriber (src/ros_python_pkg/examples/example_subscriber.py):
        Subscribes to "example_topic" and prints values as they become available.
In a separate console, inspect the nodes and their namespaces, topics, parameters using:
```
rosrun rqt rqt
```
    
Under the hood, the following sequence of events unfolds when you run "roslaunch":
1. Top-level launch file is located and run (launch/example_nodes.launch).
2. **example_nodes.launch** creates a namespace for the two nodes and their parameters (/example_nodes)
3. **example_nodes.launch** invokes **generic_node.launch** twice, once for each node.
4. **generic_node.launch** loads a config file (config/example_nodes.yaml) into the ROS parameter server the namespace (example_nodes/), then starts a node using its "bootstrapping script" (nodes/example_publisher, nodes/example_subscriber).
5. The bootstrapping script (nodes/example_publisher) creates a new instance of the node (src/ros_python_pkg/examples/example_publisher.py).

    
## Creating a new ROS node with rospy
    
Rospy is the python package providing most of the ROS API for Python programs.

## Creating the new launch configuration
    
Launching NewNode requires three things:  
  - A "bootstrapping" Python script under nodes/, which creates a new instance of NewNode  
  - A .yaml file under config/, which specifies parameters for the node  
  - A .launch file under launch/, which invokes generic_node.launch with the name of the bootstrapping script and config file.  
    
1. Copy "nodes/example_publisher"  to  "nodes/new_node".  
    Edit, replacing ExamplePublisher with the new node's class name and import path.  
    
2. Copy "config/example_nodes.yaml"  to  "config/new_node.yaml".  
    Edit, replacing with whatever parameters the node requires.  
    Currently, the following two parameters are expected, at a minimum:
    ```
    verbose: true       If true, Info and Warning messages will be printed to the console (loginfo() and logwarn() )
    rate: 10            Rate (Hz) with which the loop() function will be run. May exclude this parameter if your node does not call self.start_loop().
    ```
3. Copy "launch/example_nodes.launch"  to  "launch/new_node.launch".  
    Edit, modifying the arguments to **generic_node.launch** so it points to the new boostrapping script and config file.  
    See "launch/generic_node.launch" for all available arguments.
    
After rebuilding the Catkin workspace, the node can be launched using:
```
roslaunch your_pkg new_node.launch
```

