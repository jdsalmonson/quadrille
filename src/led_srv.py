'''
led_srv.py - LED service for the A* on the PRSG2

Jay Salmonson
9/29/2017
'''
from __future__ import print_function
import sys
from math import pi


import rospy

try:
    sys.path.append("../../ros_arduino_bridge")
    from ros_arduino_msgs.srv import DigitalWrite
except:
    pass

class LED_Srv(object):
    '''A class governing a ROS timer that blinks two LEDs at a given tempo
    (given in seconds per beat): LED #1 on the "one" beat and LED #2
    on all subsequent beats.

    t_spb (float) seconds-per-beat -- time between song beats
    i_on_frac (int) inverse fraction of a single beat for which the LED is on
    meter (int) numerator of key time signature. e.g. 3 in 3/4.
    debug (False) opt. turn-on debug print statements.
    '''
    
    def __init__(self, t_spb, i_on_frac, meter, debug = False):
        self.t_spb = t_spb #: [s] seconds per beat
        self.i_on_frac = i_on_frac #: inverse fraction of beat for which LED is on
        self.meter = meter
        self.led_dur = t_spb/i_on_frac #: [s] time to keep LED on

        self.t_start = None
        
        self.led1 = 103
        self.led2 = 104

        self.debug = debug
        
        def dummy(pin, value):
            pass

        try:
            self.led_srv = rospy.ServiceProxy('/arduino/digital_write',DigitalWrite)
            self.led_srv(pin=self.led1, value=0) # test LED to except out
        except:
            print("Warning: No LEDs available")
            self.led_srv = dummy

    def startLEDS(self):
        # Init timer for leds:
        self.timer = rospy.Timer(rospy.Duration(self.led_dur), self.onTimer)
        
    def leds_off(self):
        self.led_srv(pin=self.led1, value=0)
        self.led_srv(pin=self.led2, value=0)
        
    def onTimer(self, event):
        time = event.current_real.to_sec()
        if not self.t_start:
            self.t_start = time
        t_song = time - self.t_start

        n_beat_1  = int(t_song/self.t_spb) # need int(round()) ? 
        n_beat    = n_beat_1 + 1 #(1st is 1)
        n_measure = int(n_beat/self.meter) + 1  # measure number (1st is 1)
        # count of current measure:
        n_count   = n_beat - self.meter * (n_measure - 1)

        n_beat_seg = int(round(t_song*self.i_on_frac/self.t_spb))
        if self.debug: print("n_beat_seg = ",n_beat_seg, "n_count = ",n_count," t_song = ",t_song," residual = ",t_song/self.t_spb % self.t_spb)
        if n_beat_seg % self.i_on_frac == 0:
            if n_count == 1:
                self.led_srv(pin=self.led1, value=1) # turn on LED 1
                if self.debug: print("LED 1: ON")
            else:
                self.led_srv(pin=self.led2, value=1) # turn on LED 2
                if self.debug: print("LED 2: ON")
        elif n_beat_seg % self.i_on_frac == 1:
            #: Could simplify this; just turn off all LEDs:
            if n_count == 1:
                self.led_srv(pin=self.led1, value=0)
                if self.debug: print("LED 1: OFF")
            else:
                self.led_srv(pin=self.led2, value=0)
                if self.debug: print("LED 2: OFF")


if __name__ == "__main__":

    import time
    
    rospy.init_node("led_node")
    led_srv= LED_Srv(0.25, 4, 3, debug = True)

    led_srv.startLEDS()
    
    # LED status will be updated over the duration of the sleep...
    time.sleep(1)
