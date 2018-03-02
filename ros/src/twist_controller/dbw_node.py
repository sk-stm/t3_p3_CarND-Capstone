#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        # start with manual drive
        self._is_dbw_enabled = None
        self._current_vel_lin = None
        self._current_vel_ang = None
        self._des_lin_vel = None
        self._des_ang_vel = None

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # Create `Controller` object
        self.contoller = Controller(vehicle_mass, fuel_capacity, wheel_radius, decel_limit, accel_limit, max_steer_angle,
                                    wheel_base, steer_ratio, max_lat_accel)

        # Subscribe to all the topics you need to
        rospy.Subscriber('/twist_cmd', TwistStamped, self._get_twist_cmd)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self._set_bdw)
        rospy.Subscriber('/current_velocity', TwistStamped, self._get_current_vel)

        self.loop()

    def _set_bdw(self, is_dbw_enabled):
        self._is_dbw_enabled = is_dbw_enabled

    def _get_current_vel(self, curr_vel_twisted_stamped):
        self._current_vel_lin = curr_vel_twisted_stamped.twist.linear.x
        self._current_vel_ang = curr_vel_twisted_stamped.twist.angular.z

    def _get_twist_cmd(self, desired_speeds):
        self._des_lin_vel = desired_speeds.twist.linear.x
        self._des_ang_vel = desired_speeds.twist.angular.z

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # todo remove
            rospy.loginfo("c_vel: %s, is_dbw: %s, des_lv: %s, des_av: %s", self._current_vel_lin, self._is_dbw_enabled,
                          self._des_lin_vel, self._des_ang_vel)

            if self._current_vel_lin is not None and \
                            self._des_lin_vel is not None and \
                            self._des_ang_vel is not None and \
                            (self._is_dbw_enabled is not None):

                throttle, brake, steering = self.contoller.control(self._des_lin_vel,  # proposedlinearvelocity
                                                                   self._des_ang_vel,  # proposed angular velocity
                                                                   self._current_vel_lin  # current linear velocity
                                                                   )

                if self._is_dbw_enabled:
                   self.publish(throttle, brake, steering)
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
