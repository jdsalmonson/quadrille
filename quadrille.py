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

    def __init__(self, bpm = 60, meter = 4):
        self.bpm   = bpm
        self.meter = meter  # numerator of key time signature. e.g. 3 in 3/4.
        self.t_start = -1   # time of start of entire dance
        self.t_spb   = 60./self.bpm  # seconds per beat
        self.led_dur = 0.25*self.t_spb  # time to keep LED on
        self.led1_on = False
        self.led1_on = False
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        rospy.init_node('quadrille')

        speed = rospy.get_param("~speed", 0.5)
        turn  = rospy.get_param("~turn",  1.0)

        self.led_srv = rospy.ServiceProxy('digital_write',DigitalWrite)

    def led_time(time):
        t_song    = time - self.t_start
        n_beat    = int(t_song)/self.t_spb) + 1 #(1st is 1)
        n_measure = int(n_beat/self.meter) + 1  # measure number (1st is 1)
        # count of current measure:
        n_count   = n_beat - self.meter * (n_measure - 1)

        # led = on if time since last beat is < led duration
        if (t_song - (n_beat - 1)*self.t_spb) < self.led_dur:
            if n_count == 1:
                if self.led1_on is False:
                    self.led1_on = True
                    self.led_srv(pin=103,value=1)
                if self.led2_on is True
                    self.led2_on = False
                    self.led_srv(pin=104,value=0)
            else:
                if self.led1_on is True:
                    self.led1_on = False
                    self.led_srv(pin=103,value=0)
                if self.led2_on is False:
                    self.led2_on = True
                    self.led_srv(pin=104,value=1)
        else: # turn them both off:
            if self.led1_on is True:
                self.led1_on = False
                self.led_srv(pin=103,value=0)
            if self.led2_on is True:
                self.led2_on = False
                self.led_srv(pin=104,value=0)


    def takeStep(self, dist, n_beats, direction, theta_max):
        t_dur      = n_beats*self.spb
        vel_ave    = dist/t_dur # m/s
        shimmy_max = theta_max * pi / t_dur # max angular twist speed
        rate = rospy.Rate(10)  # Hz
        t = t0 = timer()
        # start master timer:
        if self.t_start == -1: self.t_start = t
        # publish twist 0.0 here
        while (t < t0 + t_dur):
            rate.sleep()
            t = timer()
            self.led_time(t) # blink meter LEDs
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
        dance.takeStep(0.5, 2,  1, pi/4.)
        dance.takeStep(0.5, 2, -1, pi/4.)
    except rospy.ROSInterruptException:
        print "Exception"
