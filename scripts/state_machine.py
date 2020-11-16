#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from geometry_msgs.msg import Twist, Pose2D, PoseStamped
from std_msgs.msg import String
import tf
import numpy as np
from numpy import linalg
from utils import wrapToPi
from planners import AStar, compute_smoothed_traj
from grids import StochOccupancyGrid2D
import scipy.interpolate
import matplotlib.pyplot as plt
from controllers import PoseController, TrajectoryTracker, HeadingController
from enum import Enum

from dynamic_reconfigure.server import Server
from asl_turtlebot.cfg import NavigatorConfig

# state machine modes, not all implemented
class Mode(Enum):
    EXPLORE = 0
    DELIVERY = 1

class StateMachine:
    """
    This node handles the high-level exploration and delivery operation of the 
    turtlebot
    """
    def __init__(self):
        rospy.init_node('turtlebot_state_machine', anonymous=True)
        self.mode = Mode.EXPLORE

        # current nav cmd
        self.cmd_nav = Pose2D()

	# map
	self.map = OccupancyGrid()
	self.map_meta_data = MapMetaData()

        self.traj_controller = TrajectoryTracker(self.kpx, self.kpy, self.kdx, self.kdy, self.v_max, self.om_max)
        self.pose_controller = PoseController(0., 0., 0., self.v_max, self.om_max)
        self.heading_controller = HeadingController(self.kp_th, self.om_max)
	
	# publishers
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
	self.map_meta_data_pub = rospy.Publisher('/map_meta_data', MapMetaData, queue_size=10)
	self.cmd_nav_pub = rospy.Publisher('/cmd_nav',Pose2D, queue_size=10)
	
	# subscribers
        #rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        #rospy.Subscriber('/map_metadata', MapMetaData, self.map_md_callback)
        #rospy.Subscriber('/cmd_nav', Pose2D, self.cmd_nav_callback)

        print "finished init"

        
    def publish_map(self):
        """
        Publishes map
        """
	        

        self.map_pub.publish(self.map)
	self.map_meta_data_pub.publish(self.map_meta_data)



    def publish_cmd_nav(self):
        """
        Publishes cmd_nav
        """
	        

        self.cmd_nav_pub.publish(self.cmd_nav)


    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            # try to get state information to update self.x, self.y, self.theta
            try:
                (translation,rotation) = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                self.x = translation[0]
                self.y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                self.current_plan = []
                rospy.loginfo("Navigator: waiting for state info")
                self.switch_mode(Mode.IDLE)
                print e
                pass

            # STATE MACHINE LOGIC
            # some transitions handled by callbacks
            if self.mode == Mode.EXPLORE:
                
            elif self.mode == Mode.DELIVERY:
                pass
            

            self.publish_control()
            rate.sleep()

if __name__ == '__main__':    
    nav = Navigator()
    rospy.on_shutdown(nav.shutdown_callback)
    nav.run()
