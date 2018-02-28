#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        self.base_waypoints = None

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        rospy.spin()

    def pose_cb(self, msg):
        """
        Sets the current position of the car
        :param msg: current pose of the car in PoseStamped msg format
        """
        # TODO: Implement
        self.car_pos_x = msg.pose.position.x
        self.car_pos_y = msg.pose.position.y
        self.car_pos_z = msg.pose.position.z
        self.car_ori_x = msg.pose.orientation.x
        self.car_ori_x = msg.pose.orientation.y
        self.car_ori_z = msg.pose.orientation.z
        self.car_ori_w = msg.pose.orientation.w

        # only calc final waypoints if base waypoints have been published
        if self.base_waypoints:
            idx, closest_wp_in_front_of_car = self._find_wp_in_front_of_car(self.base_waypoints)
            next_n_waypoints = []
            for i in range(LOOKAHEAD_WPS):
                next_n_waypoints.append(self.base_waypoints.waypoints[idx + i])

            self.final_waypoints_pub.publish(Lane(waypoints=next_n_waypoints))

    def waypoints_cb(self, waypoints):
        """
        Gets a giant list of waypoints in the world in Lane msg format.
        :param waypoints: waypoint list
        """
        # TODO: Implement
        self.base_waypoints = waypoints

    def _find_wp_in_front_of_car(self, waypoints):
        """
        Finds the waypoint in front of the car in the list of waypoints.
        This method assumes that the car is oriented towards tex direction of the coordinate frame.
        :param waypoints: list of waypoints
        :return: the wp in front of the car
        """
        # find closest wp to the car where wp.x > car_pos_x
        closest_wp = waypoints.waypoints[0]
        closest_idx = 0
        # TODO bet better value for 10000000
        closest_dist = 10000000.0
        for idx, wp in enumerate(waypoints.waypoints):
            curr_dist = self._wp_car_dist(wp)
            if curr_dist < closest_dist:
                closest_wp = wp
                closest_idx = idx
                closest_dist = curr_dist

        return closest_idx, closest_wp

    def _wp_car_dist(self, wp):
        """
        Calculates the distance of a waypoint to the current position of the car
        :param wp: waypoint to calc the dist to
        :return: dist between wp and car
        """
        return math.sqrt((wp.pose.pose.position.x - self.car_pos_x) ** 2 +
                         (wp.pose.pose.position.y - self.car_pos_y) ** 2 +
                         (wp.pose.pose.position.z - self.car_pos_z) ** 2)

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
        """
        Calculates the distance from wp1 to wp2
        :param waypoints: waypointlist containing all waypoints
        :param wp1: index of the first waypoint
        :param wp2: intext of the second waypoint
        :return: distance between wp1 and wp2
        """
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
