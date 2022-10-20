# Structure of a Catkin Package for Python
This repo is structured as a Catkin package containing one or more ROS nodes for Python:

```
ros_python_pkg/     # Root folder defines name of Catkin package
|   launch/                 # Launch folder holds .launch files, each of which starts one or more ROS node.
|   |
|   |   example_nodes.launch    # Launches example_publisher and example_subscriber by importing generic_node.launch
|   |   generic_node.launch     # Launches a node using a variety of configuration options
|   |
|   nodes/                  # Nodes folder holds executable python files which require ROS
|   |
|   |   example_publisher       # Python script (no .py file extension) which starts a ROS node (ExamplePublisher).
|   |   example_subscriber      # Python script (no .py file extension) which starts a ROS node (ExampleSubscriber).
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

# Initial Setup:

1. Create new Github repo as a copy of this template, and clone to your Catkin workspace's **src/** folder.
2. Rename "your_rospy_pkg/src/template_rospy_pkg/"  to  "your_rospy_pkg/src/your_rospy_pkg/".
3. Configuration: Need to edit the following files:
    - package.xml      : Edit package name, desc, author, etc. Add <depend> tags as necessary to link other Catkin packages.
    - CMakeLists.txt   : Edit package name. Need to add package dependencies here as well; same ones as in package.xml.
    - install.sh       : Edit if this package has any special installation procedures; delete if not.
    - requirements.txt : Edit if this package requires any Pip packages which are not available as standard ROS dependencies; delete if not.
 
At this point the package should build successfully using catkin-build if all dependencies are met.

# Creating a new ROS node with rospy
Rospy is the python package providing most of the ROS API for Python programs.  

# Launching the ROS node
