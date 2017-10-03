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
USE_CLASSIFIER = False


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

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


        rospy.spin()


    def visualize_tl_front(self, pose, state='WP'):
        marker = Marker()
        marker.header.frame_id = "/world"
        # marker.type = marker.ARROW
        marker.type = marker.CUBE
        if pose and state!=TrafficLight.UNKNOWN:
            marker.action = marker.ADD
        else:
            marker.action = marker.DELETE
        marker.scale.x = 10.2
        marker.scale.y = 10.2
        marker.scale.z = 400.2
        marker.color.a = 1.0
        if state == TrafficLight.GREEN:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        if state == TrafficLight.YELLOW:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        if state == TrafficLight.RED:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

        if state == 'WP':
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.scale.x = 10.2
            marker.scale.y = 10.2
            marker.scale.z = 300.2
            marker.id = 100000
        else:
            marker.id = 200000

        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1.0
        if pose:
            marker.pose.position.x = pose.position.x
            marker.pose.position.y = pose.position.y
            marker.pose.position.z = pose.position.z
        # print(" Display tl {}".format(marker))

        self.tl_front_viz.publish(marker)
        return


    def pose_cb(self, msg):
        # print(msg.pose.orientation.w)
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
            marker.scale.x = 1.2
            marker.scale.y = 1.2
            marker.scale.z = 1.2
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

            if i%50 == 0:
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

    def get_closest_waypoint(self, pose, waypts, direction=None, search_radius = 300.0):
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
        # O2 performance brute force search, consider optimizing!
        best_distance = np.Inf
        best_angle = None
        best_i = None
        if self.waypoints:
            for i, wpt in enumerate(waypts):
                # import ipdb; ipdb.set_trace()
                distance = np.sqrt((wpt.pose.pose.position.x-pose.position.x)**2 + (wpt.pose.pose.position.y-pose.position.y)**2)
                if (distance < best_distance) and (distance < search_radius):
                    # check for visibility:

                    if direction == 'F' or direction == 'R':
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
                                                wpt.pose.pose.position.y-pose.position.y, 0.0)

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

                    else:
                        angle = 0
                        best_distance = distance
                        best_angle = angle
                        best_i = i


        return best_i, best_angle, best_distance

        if not self.waypoints or not self.waypoints.waypoints:
            rospy.logerr("Waypoints empty")
            return None

        my_waypoints = self.waypoints.waypoints
        rospy.logdebug("waypoints: {}".format(len(my_waypoints)))

        pos_x = pose.position.x
        pos_y = pose.position.y

        current_distance = sys.maxsize
        current_waypoint = None

        #Lets find where we are using the euclidean distance
        for i in range(0, len(my_waypoints)):
            waypoint_pos_x = my_waypoints[i].pose.pose.position.x
            waypoint_pos_y = my_waypoints[i].pose.pose.position.y
            pos_distance = math.sqrt(math.pow(waypoint_pos_x - pos_x, 2) +
                                     math.pow(waypoint_pos_y - pos_y, 2))

            #find closest distance
            if pos_distance < current_distance:
                current_waypoint = i
                current_distance = pos_distance

        return current_waypoint

    def get_closest_light(self, waypoint_i):

        my_waypoints = self.waypoints.waypoints

        #rospy.logwarn("waypoints: {}".format(len(my_waypoints)))

        pos_x = my_waypoints[waypoint_i].pose.pose.position.x
        pos_y = my_waypoints[waypoint_i].pose.pose.position.y

        current_distance = sys.maxsize
        current_light = None

        #Lets find where we are using the euclidean distance
        for i in range(0, len(self.lights)):
            light_pos_x = self.lights[i].pose.pose.position.x
            light_pos_y = self.lights[i].pose.pose.position.y
            pos_distance = math.sqrt(math.pow(light_pos_x - pos_x, 2) +
                                     math.pow(light_pos_y - pos_y, 2))

            #find closest distance
            if pos_distance < current_distance:
                current_light = i
                current_distance = pos_distance

        return current_light, current_distance

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

        #DONE Use tranform and rotation to calculate 2D position of light in image
        # import ipdb; ipdb.set_trace()
        f = 2300
        x_offset = -30
        y_offset = 340
        fx = f
        fy = f
        piw = PyKDL.Vector(point_in_world.x,point_in_world.y,point_in_world.z)
        R = PyKDL.Rotation.Quaternion(*rot)
        T = PyKDL.Vector(*trans)
        p_car = R*piw+T

        # x = -p_car[1]/p_car[0]*fx+image_width/2
        # y = -p_car[2]/p_car[0]*fx+image_height/2
        x = -p_car[1]/p_car[0]*fx+image_width/2 + x_offset
        y = -p_car[2]/p_car[0]*fx+image_height/2+y_offset

        return (int(x), int(y))

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


        # import ipdb; ipdb.set_trace()
        # DONE: impelement projection
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        x, y = self.project_to_image_plane(light.pose.pose.position)
        if (x<0) or (y<0) or (x>=cv_image.shape[1]) or (y>=cv_image.shape[0]):
            return False


        # imm = cv2.circle(cv_image, (x,y), 10, (255,0,0), 4)
        imm = cv_image
        crop = 90
        xmin = x - crop if (x-crop) >= 0 else 0
        ymin = y - crop if (y-crop) >= 0 else 0

        # TODO:
        xmax = x + crop if (x + crop) <= imm.shape[1]-1 else imm.shape[1]-1
        ymax = y + crop if (y + crop) <= imm.shape[0]-1 else imm.shape[0]-1
        imm_cropped = imm[ymin:ymax,xmin:xmax]
        image_message = self.bridge.cv2_to_imgmsg(imm_cropped, encoding="passthrough")
        self.image_viz.publish(image_message)

        #TODO use light location to zoom in on traffic light in image

        #Get classification
        return self.light_classifier.get_classification(imm_cropped)

    def publish_image(cv_image, x, y):
        imm = cv2.circle(cv_image, (x,y), 10, (255,0,0), 4)
        image_message = self.bridge.cv2_to_imgmsg(imm, encoding="passthrough")
        self.image_viz.publish(image_message)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        tl_i = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions_plain = self.config['stop_line_positions']
        stop_line_positions = []

        if(self.pose):

            for st in stop_line_positions_plain:
                s = TrafficLight()
                s.pose.pose.position.x = st[0]
                s.pose.pose.position.y = st[1]
                s.pose.pose.position.z = 0
                s.pose.pose.orientation.x = self.pose.pose.orientation.x
                s.pose.pose.orientation.y = self.pose.pose.orientation.y
                s.pose.pose.orientation.z = self.pose.pose.orientation.z
                s.pose.pose.orientation.w = self.pose.pose.orientation.w
                stop_line_positions.append(s)
        #DONE find the closest visible traffic light (if one exists)
            tl_i, a, d =  self.get_closest_waypoint(self.pose.pose, self.lights, 'F')


        if tl_i == None:
            self.visualize_tl_front(None)
            self.visualize_tl_front(None, 0)
            return -1, TrafficLight.UNKNOWN

        # print("angle: {}".format(a))

        # import ipdb; ipdb.set_trace()
        stop_i, _, _ = self.get_closest_waypoint(self.lights[tl_i].pose.pose,
                                                    stop_line_positions)
                                                    # stop_line_positions, 'F', search_radius = 100.)
        stop_wp_i, _, _ = self.get_closest_waypoint(stop_line_positions[stop_i].pose.pose,
                                                    self.waypoints.waypoints)
        state = self.get_light_state(self.lights[tl_i])
        # state = self.lights[tl_i].state


        self.visualize_tl_front(self.waypoints.waypoints[stop_wp_i].pose.pose)
        self.visualize_tl_front(self.lights[tl_i].pose.pose, state)

        return stop_wp_i, state

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
