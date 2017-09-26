#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import tf
import cv2
import yaml
import numpy as np
import PyKDL

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        # Visualization publishers
        self.image_viz = rospy.Publisher('/image_proccessed', Image, queue_size=1)
        self.active_tl_viz = rospy.Publisher('/wpts', PoseStamped, queue_size=1)
        self.tl_viz = rospy.Publisher('tl_viz', MarkerArray)
        self.tl_front_viz = rospy.Publisher('tl_front_viz', Marker)
        self.wp_viz = rospy.Publisher('wp_viz', MarkerArray)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()


    def visualize_tl_front(self, tl_i, color='G'):
        marker = Marker()
        marker.header.frame_id = "/world"
        # marker.type = marker.ARROW
        marker.type = marker.CUBE
        if tl_i:
            marker.action = marker.ADD
        else:
            marker.action = marker.DELETE
        marker.scale.x = 100.2
        marker.scale.y = 100.2
        marker.scale.z = 300.2
        marker.color.a = 1.0
        if color == 'G':
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        if color == 'Y':
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
        if color == 'R':
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1.0
        if tl_i:
            marker.pose.position.x = self.lights[tl_i].pose.pose.position.x
            marker.pose.position.y = self.lights[tl_i].pose.pose.position.y
            marker.pose.position.z = self.lights[tl_i].pose.pose.position.z
        marker.id = 100000
        # print(" Display tl {}".format(marker))

        self.tl_front_viz.publish(marker)


    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        markerArray = MarkerArray()
        for i, wpt in enumerate(self.waypoints.waypoints):
            marker = Marker()
            marker.header.frame_id = "/world"
            # marker.type = marker.ARROW
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = .2
            marker.scale.y = .2
            marker.scale.z = .2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.x = wpt.pose.pose.orientation.x
            marker.pose.orientation.y = wpt.pose.pose.orientation.y
            marker.pose.orientation.z = wpt.pose.pose.orientation.z
            marker.pose.orientation.w = wpt.pose.pose.orientation.w
            marker.pose.position.x = wpt.pose.pose.position.x
            marker.pose.position.y = wpt.pose.pose.position.y
            marker.pose.position.z = wpt.pose.pose.position.z
            # print(" Display tl {}".format(marker))

            markerArray.markers.append(marker)


        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1

        self.wp_viz.publish(markerArray)



    def traffic_cb(self, msg):
        self.lights = msg.lights
        markerArray = MarkerArray()
        for i, wpt in enumerate(self.lights):
            marker = Marker()
            marker.header.frame_id = "/world"
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = 50.2
            marker.scale.y = 50.2
            marker.scale.z = 50.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.pose.orientation.x = wpt.pose.pose.orientation.x
            marker.pose.orientation.y = wpt.pose.pose.orientation.y
            marker.pose.orientation.z = wpt.pose.pose.orientation.z
            marker.pose.orientation.w = wpt.pose.pose.orientation.w
            marker.pose.position.x = wpt.pose.pose.position.x
            marker.pose.position.y = wpt.pose.pose.position.y
            marker.pose.position.z = wpt.pose.pose.position.z
            # print(" Display tl {}".format(marker))

            markerArray.markers.append(marker)


        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1

        self.tl_viz.publish(markerArray)

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose, waypts, direction, search_radius = 50.0):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
            waypts (list): list of waypoints to search
            direction: direction for the visibility check - 'F' for waypoint search, 'R' for traffic light search
            search_radius: search radius, [m]

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #DONE: implement

        # O2 performance brute force search, consider optimizing!
        best_distance = np.Inf
        best_angle = None
        best_i = None
        if self.waypoints:
            for i, wpt in enumerate(waypts):
                distance = np.sqrt((wpt.pose.pose.position.x-pose.position.x)**2 + (wpt.pose.pose.position.y-pose.position.y)**2)
                if (distance < best_distance) and (distance < search_radius):
                    # check for visibility:

                    # pose quaternion
                    p_q = PyKDL.Rotation.Quaternion(pose.orientation.x,
                                                pose.orientation.y,
                                                pose.orientation.z,
                                                pose.orientation.w)
                    # waypoint quaternion
                    w_q = PyKDL.Rotation.Quaternion(wpt.pose.pose.orientation.x,
                                                wpt.pose.pose.orientation.y,
                                                wpt.pose.pose.orientation.z,
                                                wpt.pose.pose.orientation.w)

                    # import ipdb; ipdb.set_trace()

                    # let's use scalar product to find the angle between the car orientation vector and car/base_point vector
                    car_orientation = p_q * PyKDL.Vector(1.0, 0.0, 0.0)
                    wp_vector = PyKDL.Vector(wpt.pose.pose.position.x-pose.position.x,
                                             wpt.pose.pose.position.y-pose.position.y,
                                             wpt.pose.pose.position.z-pose.position.z)

                    cos_angle = PyKDL.dot(car_orientation, wp_vector)/car_orientation.Norm()/wp_vector.Norm()
                    angle = np.arccos(cos_angle)

                    if direction == 'F':
                        if angle < np.pi/2.0:
                            best_distance = distance
                            best_angle = angle
                            best_i = i
                    if direction == 'R':
                        if angle > np.pi/2.0:
                            best_distance = distance
                            best_angle = angle
                            best_i = i
                    # rot = p_q * w_q.Inverse()
                    # if direction == 'F':
                    #     if np.abs(rot.GetRot()[2]) < np.pi/2.0:
                    #         best_distance = distance
                    #         best_angle = rot.GetRot()
                    #         best_i = i
                    # if direction == 'R':
                    #     if np.abs(rot.GetRot()[2]) > np.pi/2.0:
                    #         best_distance = distance
                    #         best_angle = rot.GetRot()
                    #         best_i = i

        # print(rot.GetRot())
        # print("Best i: {} @ x: {}, y:{}".format(best_i, waypts[best_i].pose.pose.position.x, waypts[best_i].pose.pose.position.y))
        # import ipdb; ipdb.set_trace()

        return best_i, best_angle, best_distance


    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use tranform and rotation to calculate 2D position of light in image

        x = 0
        y = 0

        return (x, y)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # import ipdb; ipdb.set_trace()
        # TODO: impelement projection
        # x, y = self.project_to_image_plane(light.pose.pose.position)
        x = 100
        y = 100


        imm = cv2.circle(cv_image, (50,50), 10, (255,0,0), 4)
        image_message = self.bridge.cv2_to_imgmsg(imm, encoding="passthrough")
        self.image_viz.publish(image_message)

        #TODO use light location to zoom in on traffic light in image

        #Get classification
        return self.light_classifier.get_classification(cv_image)


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        closest_tl_i = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            # car_pos_wp_i, _, _ = self.get_closest_waypoint(self.pose.pose, self.waypoints.waypoints, 'F')

        #DONE find the closest visible traffic light (if one exists)
            closest_tl_i, a, d =  self.get_closest_waypoint(self.pose.pose, self.lights, 'F')

            # if closest_tl_i:
            #     print("Closest light: {}, angle: {}, d: {}".format(self.lights[closest_tl_i], a, d))

        self.visualize_tl_front(closest_tl_i)

        light = self.get_light_state(closest_tl_i)


        if light:
            state = self.get_light_state(light)
            return light_wp, state
        # self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
