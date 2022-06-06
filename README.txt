Basic structure of a Catkin package for a ROSPY node:


template_rospy_pkg/     # Root folder defines name of Catkin package
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
|   |   template_rospy_pkg/     # Python source package should have the same name as the Catkin package
|   |   |   
|   |   |   util/                   # A python sub-package
|   |   |   |   
|   |   |   |   __init__.py             # Designates this folder as a package. Just imports everything from this folder.
|   |   |   |   config_utils.py         # Common utilities for configuring rospy nodes
|   |   |   |
|   |   |   __init__.py         # Designates this folder as a package. Just imports everything from this folder.
|   |   |   base_node.py        # Abstract class which provides basic node functionality.
|   |   |   example_publisher.py  # Example class extending BaseNode which implements a simple publisher
|   |   |   example_subscriber.py # Example class extending BaseNode which implements a simple subscriber
|   |   |   
|   |   CMakeLists.txt          # Catkin-required file which defines the build process  
|   |   install.sh              # Installs dependencies for this package (edit as required)
|   |   package.xml             # Catkin-required file which defines the package (name, dependencies on other packages, etc)  
|   |   requirements.txt        # List of python packages required to be installed by Pip
|   |   setup.py                # Installs nodes, scripts, launch files, and Python packages (src) such that other packages (and ROS) can find them
|   |
|
