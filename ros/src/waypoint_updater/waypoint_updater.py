#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import numpy as np
'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
DISTANCE_TRAFFIC_LIGHT = 50 # [in m ] when red traffic light ahead, act when closer than this distance

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.max_velocity = 10 #rospy.get_param('~max_velocity', 20)

        self.waypoints = None # read waypoints
        self.final_waypoints = None
        self.pos_point = None # Stores the waypoint index the car is closest to
        self.traffic_point = -1 # Stores the waypoint index of the closest traffic light
        self.red_light_ahead = False
        
        # Subscribers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', PoseStamped, self.obstacle_cb) # not used

        # Publisher
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        
        rospy.spin()


    def waypoints_process(self):

        # 2 options

        for ii in range(LOOKAHEAD_WPS):
            # initialize to max velocity unless there is traffic light ahead
            self.set_waypoint_velocity(self.final_waypoints, ii, self.max_velocity)

            if self.red_light_ahead:
                point_dist = self.traffic_point - self.pos_point
                chk_point_distance = (ii <  point_dist) & (point_dist > 1) & (point_dist < LOOKAHEAD_WPS)
                if chk_point_distance:
                    if self.distance(self.final_waypoints, ii, point_dist + 1) < DISTANCE_TRAFFIC_LIGHT:
                        # rospy.loginfo(self.distance(self.final_waypoints, ii, point_dist + 1))
                        self.set_waypoint_velocity(self.final_waypoints, ii, 0.0)
                    else: 
                        self.set_waypoint_velocity(self.final_waypoints, ii, self.max_velocity)
                

        self.Publish()


    def Publish(self):
        l = Lane()
        l.header = self.waypoints.header
        l.waypoints = self.final_waypoints
        self.final_waypoints_pub.publish(l)

    def waypoints_cb(self, waypoints):
        '''
        Finds the closest base waypoint position from the current car's position as an index
        Publishes the next LOOKAHEAD_WPS points
        '''
        self.waypoints = waypoints

    def pose_cb(self, msg):
        '''
        TODO:

        The first time to fine the code searches all the waypoints
        The later time only searches 100 pts ahead
        '''
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 )
        d = [] # temporary list to capture distance of waypoints from current position
        # rospy.loginfo(self.pose)

        if self.waypoints:
            for waypoint in self.waypoints.waypoints: 
                d.append(dl(waypoint.pose.pose.position, msg.pose.position))
            self.pos_point = np.argmin(d)
            self.final_waypoints = self.waypoints.waypoints[self.pos_point: self.pos_point + LOOKAHEAD_WPS +1]
            self.waypoints_process()

    def traffic_cb(self, msg):
        '''
        Reads amd processes a traffic light signal
        '''
        if (msg.data >= 0):
            self.traffic_point = msg.data
            if self.traffic_point > self.pos_point:
                self.red_light_ahead = True
            else:
                self.red_light_ahead = False            
        else:
            self.traffic_point = None
            self.red_light_ahead = False

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')