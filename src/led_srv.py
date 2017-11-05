'''
led_srv.py - LED service for the A* on the PRSG2

Jay Salmonson
9/29/2017
'''
from __future__ import print_function
import sys
from math import pi


import rospy

sys.path.append("../../ros_arduino_bridge")
from ros_arduino_msgs.srv import DigitalWrite  # * 

#from std_msgs.msg import Int16
##sys.path.append("../../stdr_simulator")
#from stdr_msgs.srv import ReadSensorPose, WriteSensorPose

ms = 1.e-3 # [milliseconds/second]

class LED_Srv(object):

    def __init__(self, t_spb, i_on_frac, meter, N_leds = 2):
        self.t_spb = t_spb #: [s] seconds per beat
        self.i_on_frac = i_on_frac #: inverse fraction of beat for which LED is on
        self.meter = meter
        self.led_dur = t_spb/i_on_frac #: [s] time to keep LED on
        #self.pubrate = 10 #: [Hz] rate of servo position publication

        self.t_start = None
        
        self.led1 = 103
        self.led2 = 104

        #self.N_leds = N_leds #: number of leds

        def dummy(pin, value):
            pass

        try:
            self.led_srv = rospy.ServiceProxy('/arduino/digital_write',foo) #DigitalWrite)
        except:
            print("Warning: No LEDs available")
            self.led_srv = dummy

        '''
        self.leds = []
        basenm = '/robot0/sonar_'
        for isrv in range(N_servos):
            servo = {}

            servo['index'] = isrv
            
            servo['read_srv'] = \
                rospy.ServiceProxy(basenm+str(isrv)+'/readpose',
                                   ReadSensorPose)
            servo['write_srv'] = \
                rospy.ServiceProxy(basenm+str(isrv)+'/writepose',
                                   WriteSensorPose)
            
            rd_srv = servo['read_srv']()
            servo['pose'] = rd_srv.pose
            servo['th_srv'] = rd_srv.pose.theta # servo->robot angle [rad]

            # The following are in the servo reference frame [deg]:
            servo['theta_ref'] = 90 # center, reference angle of servo
            servo['theta_req'] = servo['theta_ref'] # requested theta

            servo['theta'] = servo['theta_ref'] # actual servo theta

            servo['pub_theta'] = rospy.Publisher(basenm+str(isrv)+'/angle',
                                                 Int16, queue_size = 2)
            
            self.servos.append(servo)
        '''
        
    def startLEDS(self):
        #self.t_start = rospy.get_time()
        # Init timer for leds:
        print("led_dur = ",self.led_dur," ms = ",ms)
        self.timer = rospy.Timer(rospy.Duration(self.led_dur), self.onTimer)
        # Init timer for publication of servo position:
        #self.timerpub = rospy.Timer(rospy.Duration(1./self.pubrate), self.onTimerpub)
        
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

        # NEED to think about quantization:
        # n_beat = int(round(t_song/self.t_spb)) + 1 ??
        # something like this:
        n_beat_seg = int(round(t_song*self.i_on_frac/self.t_spb))
        print("n_beat_seg = ",n_beat_seg, "n_count = ",n_count," t_song = ",t_song," residual = ",t_song/self.t_spb % self.t_spb)
        #print("t_song = ",t_song)
        if n_beat_seg % self.i_on_frac == 0:
            if n_count == 1:
                self.led_srv(pin=self.led1, value=1) # turn on LED 1
                print("LED 1: ON")
            else:
                self.led_srv(pin=self.led2, value=1) # turn on LED 2
                print("LED 2: ON")
        elif n_beat_seg % self.i_on_frac == 1:
            #: Could simplify this; just turn off all LEDs:
            if n_count == 1:
                self.led_srv(pin=self.led1, value=0)
                print("LED 1: OFF")
            else:
                self.led_srv(pin=self.led2, value=0)
                print("LED 2: OFF")
        
        '''
        # could use 
        cr = event.current_real.to_sec()
        try:
            lr = event.last_real.to_sec()
        except:
            lr = cr
        dt = lr - cr
        delt = dt
        '''
        #print("{} {} {} {}".format(event.current_real, event.last_real, event.last_duration, cr - lr))  #, event.current_real - event.last_real))

        """
        # move servo thetas:
        myend = ''
        for srv in self.servos:
            if not srv['theta'] == srv['theta_req']:            
                if srv['theta'] - srv['theta_req'] > 0.9*self.delt:
                    srv['theta'] -= self.delt
                elif srv['theta'] - srv['theta_req'] < -0.9*self.delt:
                    srv['theta'] += self.delt
                else:
                    srv['theta'] = srv['theta_req']

                dth = (srv['theta'] - srv['theta_ref'])*pi/180.
                srv['pose'].theta = srv['th_srv'] + dth
                srv['write_srv'](srv['pose'])
                
            #print(" {} {}  ".format(srv['theta_req'], srv['theta']), end=myend)
            myend = "\n"
            
    def onTimerpub(self, event):
        for srv in self.servos:
            msg = Int16()
            msg.data = srv['theta']
            srv['pub_theta'].publish(msg)

    def __call__(self, id, value):
        self.servos[id]['theta_req'] = min(max(int(value), 0), 180)
        """

if __name__ == "__main__":

    import time
    
    rospy.init_node("led_node")
    led_srv= LED_Srv(0.25, 4, 3)

    led_srv.startLEDS()

    time.sleep(1)
    '''
    sss = Servo_Srv_Sim()
    sss(0,2)
    sss(1,178)
    time.sleep(1.5)
    sss(0,43)
    sss(1,87)
    time.sleep(1)
    '''
