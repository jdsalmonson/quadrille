#!/usr/bin/env python
# Quadrille - a dance node for ROS via the Twist command
#
# Usage: when a roscore (kinetic) is succesfully running:
# rosrun quadrille quadrille.py
#
# Jay Salmonson
# 11/15/2016

import roslib; roslib.load_manifest("quadrille")
import rospy

from geometry_msgs.msg import Twist

from timeit import default_timer as timer
from math import cos, pi
import sys


class quadrille(object):

    def __init__(self, bpm = 60):
        self.bpm = bpm
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        rospy.init_node('quadrille')

        speed = rospy.get_param("~speed", 0.5)
        turn  = rospy.get_param("~turn",  1.0)

    def takeStep(self, dist, n_beats, direction, theta_max):
        t_dur      = n_beats*60./self.bpm
        vel_ave    = dist/t_dur # m/s
        shimmy_max = theta_max * pi / t_dur # max angular twist speed
        rate = rospy.Rate(10)  # Hz
        t = t0 = timer()
        # publish twist 0.0 here
        while (t < t0 + t_dur):
            rate.sleep()
            t = timer()
            vel = direction * vel_ave * (1. - cos(2.*pi*(t-t0)/t_dur))
            shimmy = shimmy_max * sin(2.*pi*(t-t0)/t_dur)
            twist = Twist()
            twist.linear.x = vel
            self.pub.publish(twist)
            print (t-t0), vel, t_dur
        twist = Twist()
        twist.linear.x = 0.
        twist.angular.z = shimmy
        self.pub.publish(twist)

    def stepForward(self, dist = 0.5, n_beats = 2):
        self.takeStep(dist, n_beats, 1, 0)
    def stepBackward(self, dist = 0.5, n_beats = 2):
        self.takeStep(dist, n_beats, -1, 0)

if __name__=="__main__":

    dance = quadrille()
    try:
        dance.stepForward()
        dance.stepBackward()
        dance.takeStep(0.5, 2, 1, pi/4.)
        dance.takeStep(0.5, 2, -1, pi/4.)
    except rospy.ROSInterruptException:
        print "Exception"
