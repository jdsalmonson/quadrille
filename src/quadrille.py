#!/usr/bin/env python
# Quadrille - a dance node for ROS via the Twist command
#
# Usage: when a roscore (kinetic) is succesfully running:
# rosrun quadrille quadrille.py
#
# Jay Salmonson
# 11/15/2016

from __future__ import print_function

import roslib; roslib.load_manifest("quadrille")
import rospy

from geometry_msgs.msg import Twist

from timeit import default_timer as timer
from math import sin, cos, pi
import sys

from led_srv import LED_Srv

class Quadrille(object):
    """A ROS node to perform a dance via execution of a sequence of Twist
    commands.  If available, LEDs will be blinked to the rhythm.
    
    bpm (float) beats per minute -- tempo of song.
    meter (int) numerator of key time signature. e.g. 3 in 3/4.

    """
    
    def __init__(self, bpm = 60, meter = 4, stdr = False):
        self.bpm   = bpm
        self.meter = meter  # numerator of key time signature. e.g. 3 in 3/4.
        self.t_start = -1   # time of start of entire dance
        self.t_spb   = 60./self.bpm  # seconds per beat

        # a klooj angle muliplier because of a bug in my robot's cmd_vel:
        self.angle_factor = rospy.get_param("quadrille_angle_fac", 0.1)
        
        self.led_srv = LED_Srv(t_spb = self.t_spb, i_on_frac = 2, meter = meter)
        #self.led_srv.debug = True

        if not stdr: #: For robot
            cmd_vel = 'cmd_vel'
        else: #: For STDR simulation
            print("Quadrille configured for STDR")
            cmd_vel = 'robot0/cmd_vel'

        self.pub = rospy.Publisher(cmd_vel, Twist, queue_size = 1)
        rospy.init_node('quadrille')

        #speed = rospy.get_param("~speed", 0.5)
        #turn  = rospy.get_param("~turn",  1.0)

    def takeStep(self, dist, n_beats, direction, theta_max):
        t_dur      = n_beats*self.t_spb
        vel_ave    = dist/t_dur # m/s
        theta_max *= self.angle_factor
        shimmy_max = theta_max * pi / t_dur # max angular twist speed
        rate = rospy.Rate(20)  # Hz
        t = t0 = timer()
        # start master timer & LEDs:
        if self.t_start == -1:
            self.t_start = t
            self.led_srv.startLEDS()
        # publish twist 0.0 here
        while (t < t0 + t_dur):
            rate.sleep()
            t = timer()
            vel = direction * vel_ave * (1. - cos(2.*pi*(t-t0)/t_dur))
            shimmy = shimmy_max * sin(2.*pi*(t-t0)/t_dur)
            twist = Twist()
            twist.linear.x = vel
            twist.angular.z = shimmy
            self.pub.publish(twist)
            #print( (t-t0), vel, t_dur )
        twist = Twist()
        twist.linear.x = 0.
        twist.angular.z = 0.
        self.pub.publish(twist)
        #self.led_srv.leds_off()

    def stepForward(self, dist = 0.5, n_beats = 2):
        self.takeStep(dist, n_beats, 1, 0)
    def stepBackward(self, dist = 0.5, n_beats = 2):
        self.takeStep(dist, n_beats, -1, 0)

    def done(self):
        #rospy.sleep(1.0)
        self.led_srv.leds_off()
        self.led_srv.timer.shutdown()
        twist = Twist()
        twist.linear.x = 0.
        twist.angular.z = 0.
        self.pub.publish(twist)        
        
if __name__=="__main__":

    # Nutcracker: Waltz of Flowers
    dance = Quadrille(bpm = 190, meter = 3, stdr = False)

    print("angle_fac = ",dance.angle_factor)

    try:
        print("A")
        dance.stepForward(0.25, 6)
        dance.stepBackward(0.25, 6)
        print("B")
        dance.takeStep(0.25, 6,  1, pi/4.)
        dance.takeStep(0.25, 6, -1, pi/4.)
        print("C")
        dance.stepForward(0.25, 6)
        dance.takeStep(0.25, 6, -1, pi/4.)
        print("D")
        dance.takeStep(0.125, 6, 1, 2.*pi/4.)
        dance.takeStep(0.125, 6, 1, -2.* pi/4.)
        print("E")        
        dance.takeStep(0.5, 6, -1, pi/4.)
        dance.takeStep(0.5, 6, -1, -pi/4.)
        print("F")        
        dance.takeStep(0.125, 2, 1, pi/4.)
        dance.takeStep(0.125, 2, 1, -pi/4.)
        dance.takeStep(0.125, 2, 1, pi/4.)
        dance.takeStep(0.125, 2, 1, -pi/4.)
        dance.takeStep(0.125, 2, 1, pi/4.)
        dance.takeStep(0.125, 2, 1, -pi/4.)
        print("G")
        dance.takeStep(4.*0.125, 6, 1, 8.*pi/4.)
        dance.takeStep(4.*0.125, 6, 1, -8.* pi/4.)
        dance.takeStep(0.0, 6, 1, 8.*pi/4.)
        dance.takeStep(0.0, 6, 1, -8.* pi/4.)

        print("DONE")
        dance.done()

    except rospy.ROSInterruptException:
        print("Exception")
