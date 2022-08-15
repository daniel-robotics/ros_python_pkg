
from ros_python_pkg import BaseNode
import rospy    # Documentation: http://docs.ros.org/en/melodic/api/rospy/html/
import std_msgs.msg


class ExamplePublisher(BaseNode):

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
        
        self.start_loop()
        


    #********************************************************************************************** 
    #   PUBLISHERS
    #   Publish a message to the topic defined in the 'topic' parameter
    #   Documentation: http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
    #********************************************************************************************** 
    def init_publishers(self):
        self.loginfo("Starting publisher on %s..." % (self.params['topic']))
        self.out_msg = std_msgs.msg.UInt32()                            # The message to publish
        self.publisher = rospy.Publisher(name=self.params['topic'],     # name: topic to publish messages to
                                         data_class=type(self.out_msg), # data_class: type of message to publish
                                         queue_size=10,                 # queue_size: messages to queue before dropping old messages (decrease for lowest latency, increase if too many messages are dropped)
                                         latch=False,                   # latch: if True, the most recent message is always available to new subscribers, even if they subscribed after it was published
                                         subscriber_listener=None)      # subscriber_listener: An instance of a rospy.SubscribeListener class to receive callbacks when nodes subscribe to this topic
        
    def publish(self, msg):
        self.publisher.publish(msg)
        self.loginfo("Publishing message: %d" % msg.data)
    
    
    #********************************************************************************************** 
    #   SUBSCRIBERS
    #   Function is called when a message is received on the topic defined by the 'topic' parameter
    #   Documentation: http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
    #********************************************************************************************** 
    def init_subscribers(self):
        pass


    #********************************************************************************************** 
    #   MAIN LOOP 
    #   Function is called at the frequency defined in the 'rate' parameter (if StartMode=LOOP)
    #********************************************************************************************** 
    def loop(self):
        if not hasattr(self, 'iterations'):
            self.iterations = 0
        self.iterations += 1
        self.out_msg.data = self.iterations
        self.publish(self.out_msg)
    
     
    #********************************************************************************************** 
    #   EVENT HANDLERS
    #     on_reload(): Runs when self.params has been changed
    #     on_close():  Runs when ROS is shutting down or if CTRL-C is pressed in the terminal
    #********************************************************************************************** 
    def on_reload(self):
        pass
    
    def on_close(self):
        pass
