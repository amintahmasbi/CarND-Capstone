#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import math
import cv2
import yaml

STATE_COUNT_THRESHOLD = 1 # 3


class TLDetector(object):
    def __init__(self):
        # rospy.init_node('tl_detector', log_level = rospy.DEBUG)
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        self.light_classifier = None # compensate for late initialization

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

        # self.traffic_light_image_pub = rospy.Publisher('/traffic_light_image', Image, queue_size=1)

        self.bridge = CvBridge()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.light_classifier = TLClassifier()

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

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

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """

        closest_waypoint_distance = 100000
        closest_waypoint_idx = 0

        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

        for i in range(len(self.waypoints.waypoints)):
            dist = dl(pose.position, self.waypoints.waypoints[i].pose.pose.position)
            if dist < closest_waypoint_distance:
                closest_waypoint_idx = i
                closest_waypoint_distance = dist

        closest_waypoint = self.waypoints.waypoints[closest_waypoint_idx]

        # rospy.logdebug("next waypoint idx: %d", closest_waypoint_idx)
        theta = math.atan2(closest_waypoint.pose.pose.position.y-pose.position.y, closest_waypoint.pose.pose.position.x-pose.position.x)
        quaternion = (pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
        euler_angles = tf.transformations.euler_from_quaternion(quaternion)
        yaw_angle = euler_angles[2]

        if (math.fabs(theta - yaw_angle) > math.pi/4):
            closest_waypoint_idx += 1


        return closest_waypoint_idx

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

        if (not self.light_classifier):
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        # light_state, output = self.light_classifier.get_classification(cv_image)

        light_state = self.light_classifier.get_classification(cv_image)
        # light_state = light.state

        # image_msg = self.bridge.cv2_to_imgmsg(output, 'rgb8')
        # self.traffic_light_image_pub.publish(image_msg)
        return light_state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if self.waypoints:
            if(self.pose):
                car_position = self.get_closest_waypoint(self.pose.pose)

            closest_light = 100000
            camera_sensing_range = 200
            light_index = -1
            #Find the closest visible traffic light (if one exists)
            for idx, light_item in enumerate(self.lights):

                light_position = self.get_closest_waypoint(light_item.pose.pose)
                if car_position <= light_position:
                    dist = self.distance(self.waypoints.waypoints, car_position, light_position)
                else:
                    dist = self.distance(self.waypoints.waypoints, light_position, car_position)
                    dist = len(self.waypoints.waypoints) - dist

                # rospy.logdebug("next light distance: %d | %d | %d", car_position, light_position, int(dist))
                if dist <= camera_sensing_range:
                    closest_light = dist
                    light = light_item
                    light_index = idx

            if light:
                state = self.get_light_state(light)

                stop_line = Pose()
                stop_line.position.x = stop_line_positions[light_index][0]
                stop_line.position.y = stop_line_positions[light_index][1]
                stop_line.position.z = 0
                line_position = self.get_closest_waypoint(stop_line)
                # rospy.logdebug("Next light : %d --> %d ", car_position, line_position)
                # rospy.logdebug("Light status: %d ", int(state))
                return line_position, state

        return -1, TrafficLight.UNKNOWN

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            # TODO: list index out of range for looping around track
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
