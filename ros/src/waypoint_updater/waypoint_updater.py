#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import copy
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

LOOKAHEAD_WPS = 100  # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.my_base_waypoints = None
        self.next_n_waypoints = []
        self.next_n_waypoint_glob_idxs = []
        self.stop_at_tl = False
        self.car_wp_idx = None
        self.car_pos_x = 0
        self.car_pos_y = 0
        self.car_pos_z = 0
        self.car_ori_x = 0
        self.car_ori_y = 0
        self.car_ori_z = 0
        self.car_ori_w = 0

        rospy.spin()

    def pose_cb(self, msg):
        """
        Sets the current position of the car
        :param msg: current pose of the car in PoseStamped msg format
        """
        self.car_pos_x = msg.pose.position.x
        self.car_pos_y = msg.pose.position.y
        self.car_pos_z = msg.pose.position.z
        self.car_ori_x = msg.pose.orientation.x
        self.car_ori_x = msg.pose.orientation.y
        self.car_ori_z = msg.pose.orientation.z
        self.car_ori_w = msg.pose.orientation.w

        # only calc final waypoints if base waypoints have been published
        if self.base_waypoints and not self.stop_at_tl:
            self._calc_next_waypoints()

            self.final_waypoints_pub.publish(Lane(waypoints=self.next_n_waypoints))

    def _calc_next_waypoints(self):
        """
        TODO
        :return:
        """
        #rospy.loginfo("wp_updater: wp_idx before inital search %s", self.car_wp_idx)
        self.car_wp_idx = self._find_wp_in_front_of_car()
        #rospy.loginfo("wp_updater: wp_idx after inital search %s", self.car_wp_idx)

        if not self.next_n_waypoints:
            # initial fill
            #rospy.loginfo("wp_updater: inital fill wps no tl")
            for i in range(LOOKAHEAD_WPS):
                next_idx = self.car_wp_idx + i
                next_wp = copy.deepcopy(self.base_waypoints.waypoints[next_idx])
                self.next_n_waypoints.append(next_wp)
                self.next_n_waypoint_glob_idxs.append(next_idx)
                # start smoothly when standing
                current_vel = self.get_waypoint_velocity(self.base_waypoints.waypoints[self.car_wp_idx])
                proposed_vel = self.get_waypoint_velocity(self.base_waypoints.waypoints[self.car_wp_idx + LOOKAHEAD_WPS])
                next_wp_vel = current_vel + (current_vel - proposed_vel) / LOOKAHEAD_WPS
                self.set_waypoint_velocity(self.next_n_waypoints, -1, next_wp_vel)
        else:
            #rospy.loginfo("wp_updater: update available wps no tl")
            # if waypoints are available
            # find current wp in list of next wps starting at the beginning
            i = 0
            while i < LOOKAHEAD_WPS and self.next_n_waypoint_glob_idxs[i] < self.car_wp_idx:
                # remove all entries up to that index
                self.next_n_waypoints.pop(0)
                self.next_n_waypoint_glob_idxs.pop(0)
                # append as many as removed at the end
                next_wp = copy.deepcopy(self.base_waypoints.waypoints[self.car_wp_idx + LOOKAHEAD_WPS + i])
                self.next_n_waypoints.append(next_wp)
                self.next_n_waypoint_glob_idxs.append(self.car_wp_idx + LOOKAHEAD_WPS + i)
                i += 1

    def waypoints_cb(self, waypoints):
        """
        Gets a giant list of waypoints in the world in Lane msg format.
        :param waypoints: waypoint list
        """
        self.base_waypoints = waypoints
        self.my_base_waypoints = copy.deepcopy(waypoints)

    def _find_wp_in_front_of_car(self):
        """
        Finds the waypoint in front of the car in the list of waypoints.
        This method assumes that the car is oriented towards tex direction of the coordinate frame.
        :return: the wp in front of the car
        """
        # find closest wp to the car where wp.x > car_pos_x
        closest_idx = self.car_wp_idx
        i = 1
        closest_dist = float("inf")

        if self.car_wp_idx is not None:
            closest_dist = self._wp_car_dist(self.base_waypoints.waypoints[self.car_wp_idx])
            next_wp_car_dist = self._wp_car_dist(self.base_waypoints.waypoints[self.car_wp_idx + i])
            while closest_dist > next_wp_car_dist:
                closest_dist = next_wp_car_dist
                i += 1
                next_wp_car_dist = self._wp_car_dist(self.base_waypoints.waypoints[self.car_wp_idx + i])
                closest_idx = self.car_wp_idx + i
        else:
            # initial search in all the waypoints
            for idx, wp in enumerate(self.base_waypoints.waypoints):
                curr_dist = self._wp_car_dist(wp)
                if curr_dist < closest_dist:
                    closest_idx = idx
                    closest_dist = curr_dist

        return closest_idx

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
        stop_wp_idx = msg.data
        # if the index is -1 this means there are no traffic lights to stop at
        if stop_wp_idx == -1:
            if self.stop_at_tl:
                self.stop_at_tl = False
                # reset next_waypoints
                self.next_n_waypoints = []
                self.next_n_waypoint_glob_idxs = []
                rospy.loginfo("wp_updater: green light")
            return

        rospy.loginfo("wp_updater: red light")
        self.car_wp_idx = self._find_wp_in_front_of_car()

        rospy.loginfo("wp_updater: car_idx %s, stop_idx: %s", self.car_wp_idx, stop_wp_idx)
        # TODO check next statement
        # if initially the tl is green and the camera is on but the autonomous mode not
        if self.base_waypoints:
            num_wp_until_stop = stop_wp_idx - self.car_wp_idx

            reinit = False
            if not self.next_n_waypoints:
                current_vel = self.get_waypoint_velocity(self.base_waypoints.waypoints[self.car_wp_idx])
                self.next_n_waypoints = []
                self.next_n_waypoint_glob_idxs = []
                reinit = True
            else:
                current_vel = self.get_waypoint_velocity(self.next_n_waypoints[0])

            wp_vel = current_vel
            if self.car_wp_idx < stop_wp_idx:
                # normal case we stop before traffic light
                rospy.loginfo("wp_updater: we're stoping in front of red light")
                for i in range(LOOKAHEAD_WPS):
                    wp_idx = self.car_wp_idx + i
                    if reinit:
                        next_wp = copy.deepcopy(self.base_waypoints.waypoints[wp_idx])
                        self.next_n_waypoints.append(next_wp)
                        self.next_n_waypoint_glob_idxs.append(wp_idx)
                        if wp_idx < stop_wp_idx:
                            # gradually decrease desired velocity
                            wp_vel -= current_vel / num_wp_until_stop
                            self.set_waypoint_velocity(self.next_n_waypoints, i, max(wp_vel,0))
                            #rospy.loginfo("wp_updater: wp_idx: %s, waypoint_vel: %s", wp_idx, wp_vel)
                            #rospy.loginfo("wp_updater: current_wp_idx: %s", self.car_wp_idx)
                        else:
                            # if you drive further then intended just stop!
                            self.set_waypoint_velocity(self.next_n_waypoints, i, 0)
                    else:
                        if self.next_n_waypoint_glob_idxs[i] < self.car_wp_idx:
                            # remove all entries up to that index
                            self.next_n_waypoints.pop(0)
                            self.next_n_waypoint_glob_idxs.pop(0)
                            # append as many as removed at the end
                            next_wp = copy.deepcopy(self.base_waypoints.waypoints[self.car_wp_idx + LOOKAHEAD_WPS + i])
                            self.next_n_waypoints.append(next_wp)
                            self.next_n_waypoint_glob_idxs.append(self.car_wp_idx + LOOKAHEAD_WPS + i)
                            self.set_waypoint_velocity(self.next_n_waypoints, i, 0)

                self.set_waypoint_velocity(self.next_n_waypoints, -1, 0)
            else:
                # we just drove over the stop line ut the traffic light is still red
                rospy.loginfo("wp_updater: we just drove over a red light")
                self.next_n_waypoints = []
                for i in range(self.car_wp_idx, self.car_wp_idx + LOOKAHEAD_WPS):
                    idx_next_wp_list = i - self.car_wp_idx
                    next_wp = copy.deepcopy(self.base_waypoints.waypoints[i])
                    self.next_n_waypoints.append(next_wp)
                    self.set_waypoint_velocity(self.next_n_waypoints, idx_next_wp_list, 0)

            # TODO remove
            #rospy.loginfo("wp_updater: num wp until stop: %s; current vel: %s", num_wp_until_stop, current_vel)

            self.stop_at_tl = True
            self.final_waypoints_pub.publish(Lane(waypoints=self.next_n_waypoints))

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint_idx, velocity):
        waypoints[waypoint_idx].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        """
        Calculates the distance from wp1 to wp2
        :param waypoints: waypointlist containing all waypoints
        :param wp1: index of the first waypoint
        :param wp2: intext of the second waypoint
        :return: distance between wp1 and wp2
        """
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
