#!/usr/bin/env python

from __future__ import division

import rospy

from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import tf

import sys
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

'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 1.0
MAX_ACCEL = 1.0

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Add other member variables you need below
        max_velocity_kmh = rospy.get_param('/waypoint_loader/velocity', 40)
        self.max_velocity = max_velocity_kmh * 1000. / 3600.
        self.pose = None
        self.base_waypoints = None
        self.velocity = None
        self.previous_velocity = None
        self.dt = None
        self.is_accelerating = False
        self.is_stopping = False
        self.traffic_waypoint = -1
        self.last_traffic_waypoint = -1
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

        # rospy.logdebug("next waypoint idx: %d", closest_waypoint_idx)
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
            # update final_waypoints if there is a traffic light
            if (not self.is_stopping) and (self.traffic_waypoint != -1): # Traffic light turned red
                final_waypoints = self.prepare_to_stop(next_waypoint_idx, final_waypoints)

            elif  (self.is_stopping) and (not self.is_accelerating) and (self.traffic_waypoint == -1): # Traffic light turned green or vehicle passed traffic light
                # rospy.logwarn("==========================================================================")
                final_waypoints = self.prepare_to_move(next_waypoint_idx, final_waypoints)

            # else: # No traffic light or no change in light
                # for index in range(LOOKAHEAD_WPS):
                    # self.set_waypoint_velocity(final_waypoints, index, self.max_velocity)

            rospy.logdebug(final_waypoints[0].twist.twist.linear.x)
            msg = Lane()
            msg.header.seq = self.seq_num
            msg.header.stamp = rospy.Time.now()
            # msg.header.frame_id = ''
            msg.waypoints = final_waypoints

            #rospy.logdebug("next waypoint idx: %d", next_waypoint_idx)
            self.final_waypoints_pub.publish(msg)
        pass

    def prepare_to_stop(self, next_waypoint, waypoints):
        """adjust the target velocities for the waypoints leading up to red traffic lights
        in order to bring the vehicle to a smooth and full stop

        :next_waypoint: index of closest_waypoint in front of the vehicle
        :waypoints: TODO
        :returns: TODO

        """
        traffic_waypoint = self.traffic_waypoint
        distance_to_light = self.distance(self.base_waypoints, next_waypoint, traffic_waypoint)

        # lookahead_distance = self.distance(self.base_waypoints, next_waypoint, next_waypoint+LOOKAHEAD_WPS)

        # rospy.logdebug("lookahead: %f", lookahead_distance)
        if traffic_waypoint < next_waypoint or traffic_waypoint-next_waypoint > LOOKAHEAD_WPS/2:
            self.is_stopping = False
            return waypoints
        else:
            self.is_stopping = True
            self.is_accelerating = False
            # rospy.logdebug("d2l: %f", distance_to_light)
            # if self.previous_velocity:
                # acceleration = (self.velocity.twist.linear.x - self.previous_velocity.twist.linear.x)/self.dt.to_sec() #NOTE: negative if braking
            # else:
                # acceleration = 0.

            if math.fabs(self.velocity.twist.linear.x) >= 1.:
            # if math.fabs(self.velocity.twist.linear.x) >= 0: # always true
                # start_state = np.array([0., self.velocity.twist.linear.x, acceleration])
                # end_state = np.array([distance_to_light, 0., 0.])

                # NOTE: time2stop could also be derived based on the Jerk requirements of the project
                # ballpark_time2stop = distance_to_light / 1. # Constant speed aproximation

                # jmt_coeffs = self.jerk_minimization_trajectory(start_state, end_state, ballpark_time2stop)

                # rospy.logdebug(jmt_coeffs)
                for index in range(traffic_waypoint-next_waypoint):
                    # distance2index = self.distance(self.base_waypoints, next_waypoint, next_waypoint+index)
                    # velocity_of_index = self.poly_first_derivitive_eval(jmt_coeffs, distance2index)

                    index2light = self.distance(self.base_waypoints, next_waypoint + index, traffic_waypoint)
                    if index2light >= 5.:
                        velocity_of_index = self.get_waypoint_velocity(self.base_waypoints[next_waypoint+index]) * ( float(index2light) / distance_to_light )
                    else:
                        velocity_of_index = 0.

                    if velocity_of_index < 1.:
                        velocity_of_index = 0.
                    self.set_waypoint_velocity(waypoints, index, velocity_of_index)

            else:
                # self.is_accelerating = False
                for index in range(traffic_waypoint-next_waypoint):
                    self.set_waypoint_velocity(waypoints, index, 0.)

            return waypoints

    def prepare_to_move(self, next_waypoint, waypoints):
        """Adjust the target velocities for the waypoint leading up to traffic lights
        in order to bring the vehicle to normal speed

        :waypoints: TODO
        :returns: TODO

        """

        # if self.previous_velocity:
            # acceleration = (self.velocity.twist.linear.x - self.previous_velocity.twist.linear.x)/self.dt.to_sec()
        # else:
            # acceleration = 0.

        # rospy.logdebug(self.get_waypoint_velocity(self.base_waypoints[next_waypoint]))
        if (self.velocity.twist.linear.x) < self.max_velocity:
            self.is_stopping = False
            self.is_accelerating = True

            # ballpark_time2moving = 2 #2 #9. / (self.get_waypoint_velocity(waypoints[0]) - self.velocity.twist.linear.x) # constant acceleration to full speed
            # distance2moving = 0.5 * 9. * (ballpark_time2moving**2) + self.velocity.twist.linear.x*ballpark_time2moving

            # start_state = np.array([0., self.velocity.twist.linear.x, acceleration])
            # end_state = np.array([distance2moving, self.get_waypoint_velocity(waypoints[0]), 0.])

            # jmt_coeffs = self.jerk_minimization_trajectory(start_state, end_state, ballpark_time2moving)

            for index in range(LOOKAHEAD_WPS):
                distance2index = self.distance(self.base_waypoints, next_waypoint, next_waypoint+index)
                # velocity_of_index = self.poly_first_derivitive_eval(jmt_coeffs, distance2index)
                velocity_of_index = 1. + 2. * MAX_ACCEL * float(distance2index)

                self.set_waypoint_velocity(waypoints, index, min(velocity_of_index, self.max_velocity))
        # else:
            # self.is_accelerating = False

        return waypoints

    def decelerate(self, waypoints):
        last = waypoints[-1]
        last.twist.twist.linear.x = 0.
        for wp in waypoints[:-1][::-1]:
            dist = self.euclidean_distance(wp.pose.pose.position, last.pose.pose.position)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
        return waypoints

    def accelerate(self, waypoints):
        last = waypoints[-1]
        last.twist.twist.linear.x = self.max_velocity
        for wp in waypoints[:-1][::-1]:
            dist = self.euclidean_distance(wp.pose.pose.position, last.pose.pose.position)
            vel = math.sqrt(2 * MAX_ACCEL * dist)
            if vel < 1.:
                vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
        return waypoints

    def euclidean_distance(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    def poly_eval(self, coeffs, x):
        """Calculate the value of a polynomial function at 'x'

        :coeffs: TODO
        :x: TODO
        :returns: TODO

        """
        y = 0.
        for ind, coeff in enumerate(coeffs):
            i = ind+1
            y += coeff * (x ** i)

        return y

    def poly_first_derivitive_eval(self, coeffs, x):
        """Calculate the value of first derivation of a polynomial function at 'x'

        :coeffs: TODO
        :x: TODO
        :returns: TODO

        """
        y = 0.
        for ind, coeff in enumerate(coeffs):
            i = ind+1 # Python enumerates from 0
            y += i * coeff * (x ** (i-1))

        return y

    def jerk_minimization_trajectory(self, start_state, end_state, t):
        """Calculate the Jerk Minimization Trajectory that connects the
        initial state to the end state in time t

        :start_state: The vehicles start location given as a list corresponding to initial values of [s, s_dot, s_double_dot]
        :end_state: The desired end state for vehicle. Similar to start state
        :t: The duration, in seconds, over which this maneuver should occur
        :returns: 6 Coeffients of the 5th order JMT polynomial 

        """
        coeff = np.zeros((6,))

        coeff[0] = start_state[0]
        coeff[1] = start_state[1]
        coeff[2] = 0.5 * start_state[2]

        t2 = t**2
        t3 = t**3
        t4 = t**4
        t5 = t**5

        tmp = np.array([[t3, t4, t5],
                        [3*t2, 4*t3, 5*t4],
                        [6*t, 12*t2, 20*t3]])


        v = np.zeros((3,))
        v[0] = end_state[0] - (start_state[0] + start_state[1] * t + 0.5 * start_state[2] * t2)
        v[1] = end_state[1] - (start_state[1] + start_state[2] * t)
        v[2] = end_state[2] - start_state[2]

        if np.linalg.cond(tmp) < 1./sys.float_info.epsilon:
            co = np.linalg.inv(tmp).dot(v)
            coeff[3] = co[0]
            coeff[4] = co[1]
            coeff[5] = co[2]

        return coeff

    def pose_cb(self, msg):
        self.seq_num = msg.header.seq
        self.pose = msg.pose
        self.update_final_waypoints()
        pass

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints.waypoints
        rospy.logdebug("Base waypoints received: %d", len(self.base_waypoints))
        pass

    def traffic_cb(self, msg):
        # Callback for /traffic_waypoint message.
        self.last_traffic_waypoint = self.traffic_waypoint
        self.traffic_waypoint = msg.data
        # if self.traffic_waypoint != -1:
            # rospy.logdebug(self.traffic_waypoint)
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def velocity_cb(self, msg):
        """Callback for /current_velocity
        """
        self.previous_velocity = self.velocity
        self.velocity = msg
        if self.dt:
            self.dt = self.velocity.header.stamp - self.previous_velocity.header.stamp
        else:
            self.dt = 1 # first received msg from velocity topic

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
