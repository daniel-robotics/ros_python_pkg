
from ros_python_pkg import BaseNode
import rospy    # Documentation: http://docs.ros.org/en/melodic/api/rospy/html/
import std_msgs.msg


class ExampleSubscriber(BaseNode):

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
       
        super().__init__(name,
                        param_namespace,
                        package_path,
                        clear_params_on_exit)
        
        self.start_spin()


    #********************************************************************************************** 
    #   PUBLISHERS
    #   Publish a message to the topic defined in the 'topic' parameter
    #   Documentation: http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
    #********************************************************************************************** 
    def init_publishers(self):
        pass

    #********************************************************************************************** 
    #   SUBSCRIBERS
    #   Function is called when a message is received on the topic defined by the 'topic' parameter
    #   Documentation: http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
    #********************************************************************************************** 
    def init_subscribers(self):
        self.loginfo("Starting subscriber on %s..." % (self.params['topic']))
        self.in_msg = std_msgs.msg.UInt32()                                 # The most recently received message
        self.subscriber = rospy.Subscriber(name=self.params['topic'],                 # name: topic to listen to
                                           data_class=type(self.in_msg),    # data_class: message type for this topic 
                                           callback=self.message_received)  # callback: function to call when a message is received on this topic
        
    def message_received(self, msg):
        self.in_msg=msg 
        self.loginfo("Received message: %d" % msg.data)


    #********************************************************************************************** 
    #   MAIN LOOP 
    #   Function is called at the frequency defined in the 'rate' parameter (if StartMode=LOOP)
    #********************************************************************************************** 
    def loop(self):
        pass


    #********************************************************************************************** 
    #   EVENT HANDLERS
    #     on_reload(): Runs when self.params has been changed
    #     on_close():  Runs when ROS is shutting down or if CTRL-C is pressed in the terminal
    #********************************************************************************************** 
    def on_reload(self):
        pass
    
    def on_close(self):
        pass
        
