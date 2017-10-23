#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import tf

import math

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


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        # rospy.Subscriber('/traffic_waypoint', Int32, self.waypoints_cb)
        # rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.seq_num = 0
        rospy.spin()

    def find_closest_waypoint(self):
        """TODO: Docstring for find_closest_waypoint.
        :returns: TODO

        """
        closest_waypoint_distance = 100000;
        closest_waypoint_idx = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

        for i in range(len(self.base_waypoints)):
            dist = dl(self.pose.position, self.base_waypoints[i].pose.pose.position)
            if dist < closest_waypoint_distance:
                closest_waypoint_idx = i
                closest_waypoint_distance = dist

        return closest_waypoint_idx

    def find_next_waypoint(self):
        """Find index of closest waypoint
        :returns: index (Int32)
        """
        closest_waypoint_idx = self.find_closest_waypoint()
        closest_waypoint = self.base_waypoints[closest_waypoint_idx]

        rospy.logdebug("next waypoint idx: %d", closest_waypoint_idx)
        theta = math.atan2(closest_waypoint.pose.pose.position.y-self.pose.position.y, closest_waypoint.pose.pose.position.x-self.pose.position.x)
        quaternion = (self.pose.orientation.w, self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z)
        euler_angles = tf.transformations.euler_from_quaternion(quaternion)
        yaw_angle = euler_angles[2]

        if (math.fabs(theta - yaw_angle) > math.pi/4):
            closest_waypoint_idx += 1


        return closest_waypoint_idx

    def update_final_waypoints(self):
        """Prepare and publish final waypoints

        """
        if self.base_waypoints is not None:
            base_waypoints_len = len(self.base_waypoints)
            next_waypoint_idx = self.find_next_waypoint()
            final_waypoints = []
            for index in range(next_waypoint_idx, next_waypoint_idx + LOOKAHEAD_WPS):
                final_waypoints.append(self.base_waypoints[index%base_waypoints_len])

            msg = Lane()
            msg.header.seq = self.seq_num
            msg.header.stamp = rospy.Time.now()
            # msg.header.frame_id = ''
            msg.waypoints = final_waypoints

            #rospy.logdebug("next waypoint idx: %d", next_waypoint_idx)
            self.final_waypoints_pub.publish(msg)
        pass

    def pose_cb(self, msg):
        # TODO: Implement
        self.seq_num = msg.header.seq
        self.pose = msg.pose
        self.update_final_waypoints()
        pass

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints.waypoints
        rospy.logdebug("Base waypoints received: %d", len(self.base_waypoints))
        # TODO: Implement
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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
