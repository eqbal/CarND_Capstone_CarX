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

LOOKAHEAD_WPS = 150 # Number of waypoints we will publish. You can change this number
LIMIT_TRAFFIC_LIGHT = 20 # [in m ] when red traffic light ahead, act when closer than this distance
LIMIT_DECELERATE = 40 # distance to start decelerating
MAX_DECEL = 5 # max deceleration in m/s^2. this is just an indicative value from the loader node

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        self.waypoints = None # read waypoints
        self.final_waypoints = None
        self.pos_point = None # Stores the waypoint index the car is closest to
        self.traffic_point = -1 # Stores the waypoint index of the closest traffic light
        self.red_light_ahead = False
        self.lookahead_wps = 0
        self.max_velocity = 1 # m/s


        # Subscribers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', PoseStamped, self.obstacle_cb) # not used



        # Publisher
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        rospy.spin()


    def waypoints_process(self):
        #rospy.logwarn("me...")
        for ii in range(self.lookahead_wps):
            # initialize to max velocity unless there is traffic light ahead
            velocity = self.max_velocity
            if self.red_light_ahead:
                point_dist = self.traffic_point - self.pos_point
                chk_point_distance = (ii <  point_dist) & (point_dist > 1) & (point_dist < self.lookahead_wps)
                if chk_point_distance:
                    distance_traffic_light = self.distance(self.final_waypoints, ii, point_dist + 1)
                    if (distance_traffic_light < LIMIT_TRAFFIC_LIGHT):
                        velocity = 0.0
                  #  elif distance_traffic_light < LIMIT_DECELERATE:
                  #      velocity = max(self.max_velocity - math.sqrt(2*MAX_DECEL*distance_traffic_light)* 3.6, 0)

                #if ii % 10 == 0:
                #    rospy.logwarn('i: %d : Vel: %.3f', ii, velocity)

            self.set_waypoint_velocity(self.final_waypoints, ii, velocity)

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
        self.waypoints_size = np.shape(waypoints.waypoints)[0]
        self.lookahead_wps = min(LOOKAHEAD_WPS, self.waypoints_size//2)
        # rospy.logwarn("Total waypoints {}".format(self.waypoints_size))
        self.max_velocity = self.get_waypoint_velocity(waypoints.waypoints[0]) # m/s
        # rospy.logwarn("Max Velocity {}".format(self.max_velocity))


    def pose_cb(self, msg):
        '''
        Return closest waypoint
        '''
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 )
        d = [] # temporary list to capture distance of waypoints from current position
        if self.waypoints:
            for waypoint in self.waypoints.waypoints:
                d.append(dl(waypoint.pose.pose.position, msg.pose.position))
            self.pos_point = np.argmin(d)

            # Check to implement circular list
            if self.pos_point + self.lookahead_wps +1 > self.waypoints_size:
                list_end = self.pos_point + self.lookahead_wps +1 - self.waypoints_size
                self.final_waypoints = self.waypoints.waypoints[self.pos_point:] + self.waypoints.waypoints[:list_end]
            else:
                self.final_waypoints = self.waypoints.waypoints[self.pos_point: self.pos_point + self.lookahead_wps +1]

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
