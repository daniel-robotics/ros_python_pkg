
from abc import ABC, abstractmethod
import rospy    # Documentation: http://docs.ros.org/en/melodic/api/rospy/html/


class BaseNode(ABC):

    #********************************************************************************************** 
    #   INIT
    #       name:
    #           base name to register node with (no spaces, slashes, special characters)
    #       param_namespace:
    #           loads all parameters from the given namespace path. If param_namespace='~', loads parameters from this node's private namespace
    #       package_path:
    #           path to root of Catkin package
    #       clear_params_on_exit:
    #           If True, attempts to delete all parameters in param_namespace on exit
    #********************************************************************************************** 
    def __init__(self, name, param_namespace, package_path, clear_params_on_exit):

        # Register a node with the ROS Master server and register a shutdown hook
        #    - only one node allowed per process
        rospy.init_node(name)    
        rospy.on_shutdown(self.shutdown)        # Function is called when ROS shuts down 

        self.name = rospy.get_name()            # The fully-qualified name may be different from the one provided to init_node
        self.namespace = rospy.get_namespace()
        self.params={}
        self.param_namespace = param_namespace
        self.package_path = package_path
        self.clear_params_on_exit = clear_params_on_exit
        self.rate = None
        
        # Read parameters (as a dict) from the ROS parameter server
        self.reload_params(param_namespace)

        self.logwarn("Initializing node...", force=True)
        
        if self.params["verbose"] is True:
            self.loginfo("Starting node with parameters:")
            print(self.params)

        # Create publishers and subscribers
        self.init_publishers()
        self.init_subscribers()

        self.logwarn("Node Initialized", force=True)


    # Runs self.loop() at the rate set by the parameter, until ROS shuts down or CTRL-C is pressed in the terminal
    # IF rate==0, loop as fast as possible without sleeping
    def start_loop(self):
        while not rospy.is_shutdown():
            self.loop()
            if self.rate is not None:
                try:
                    if self.rate.remaining().secs < 0:
                        self.logwarn("Loop running too slowly to keep up with fixed rate %d Hz" % (self.params['rate']))
                    if not rospy.is_shutdown():
                        self.rate.sleep()
                except rospy.ROSInterruptException:
                    pass
    
    # Alternative start: rospy.spin() blocks until ROS shuts down (but subscriber callbacks still run)
    def start_spin(self):
        self.loginfo("Idling until exit")
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

    # Download this node's parameters from the parameter server
    def reload_params(self, param_namespace=None):
        if param_namespace is not None:
            self.param_namespace = param_namespace
        
        self.params = rospy.get_param(self.param_namespace)
        if "rate" in self.params:
            self.set_rate(self.params['rate'])
        else:
            self.rate = None
        
        self.on_reload()

        return self.params
    
    
    # Create a Rate object to govern the main loop frequency 
    #   IF rate_hz<=0, loop as fast as possible without sleeping
    def set_rate(self, rate_hz):
        if rate_hz > 0:
            self.loginfo("Setting frequency to %.2f Hz." % (rate_hz))
            self.params['rate'] = rate_hz
            self.rate = rospy.Rate(rate_hz)
        else:
            self.logwarn("Parameter \"rate\" = 0; frequency unrestricted.")
            self.logwarn("This may be very taxing on the CPU and/or network.")
            self.params['rate'] = 0
            self.rate = None


     
    #********************************************************************************************** 
    #   PUBLISHERS
    #   Documentation: http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
    #********************************************************************************************** 
    @abstractmethod
    def init_publishers(self):
        pass
    
    #********************************************************************************************** 
    #   SUBSCRIBERS
    #   Documentation: http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
    #********************************************************************************************** 
    @abstractmethod
    def init_subscribers(self):
        pass

    #********************************************************************************************** 
    #   MAIN LOOP 
    #   Function is called at the frequency defined in the 'rate' parameter (if StartMode=LOOP)
    #********************************************************************************************** 
    @abstractmethod
    def loop(self):
        pass
    
    #********************************************************************************************** 
    #   EVENT HANDLERS
    #     on_reload(): Runs when self.params has been changed
    #     on_close():  Runs when ROS is shutting down or if CTRL-C is pressed in the terminal
    #********************************************************************************************** 
    @abstractmethod
    def on_reload(self):
        pass
    
    @abstractmethod
    def on_close(self):
        pass

    #********************************************************************************************** 
    #   LOGGING
    #********************************************************************************************** 
    # Logs a Message string to the console (only if parameter "verbose" is True or force=True is provided)
    def loginfo(self, msg, force=False):
        if self.params['verbose'] is True or force is True:
            rospy.loginfo("[%s] %s" % (self.name, msg) )
    
    # Logs a Warning string to the console (only if parameter "verbose" is True or force=True is provided)
    def logwarn(self, msg, force=False):
        if self.params['verbose'] is True or force is True:
            rospy.logwarn("[%s] %s" % (self.name, msg) )
   
    # Logs an Error string to the console (only if parameter "verbose" is True or force=True is provided)
    def logerr(self, msg, force=False):
        if self.params['verbose'] is True or force is True:
            rospy.logerr("[%s] %s" % (self.name, msg) )

    # Shutdown hook: runs when ROS is shutting down
    def shutdown(self):
        try:
            self.logwarn("Terminating node...", force=True)
            self.on_close()
            if self.clear_params_on_exit is True:
                rospy.delete_param(self.param_namespace)
                rospy.signal_shutdown("")
        except:
            pass
        