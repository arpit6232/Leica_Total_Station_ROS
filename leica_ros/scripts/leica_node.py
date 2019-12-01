#!/usr/bin/env python
'''
Python node to talk to a Leica Totalstation 1200 series to use it as a tracker system
Based off of 
https://github.com/rvermeiren/Leica-GeoCOM-for-drone-tracking.git
and
https://github.com/georgwi/leica_ros_sph.git

Modified to support a service interface to initialise the TS without console interaction
But it uses the same GeoCOM Python interface (just in a nice thread-safe manner)
The GeoCOM interface uses a global (!) serial port object - ugh


'''
from __future__ import division
from __future__ import print_function

from future import standard_library 
standard_library.install_aliases()
from builtins import str
from past.utils import old_div
from builtins import object
import rospy
import GeoCom
import actionlib

from leica_ros.msg import *
from leica_ros.srv import *
from geometry_msgs.msg import PointStamped, Twist
from std_srvs.srv import *

from optparse import OptionParser
import math
from math import sin,cos
from queue import *
from threading import *
import pdb

''' 
Since ROS service handlers are called in a separate thread, there needs to be a thread-safe way
to get requests into a queue and be able to make intelligent return codes
A queue would handle the first part easily enough - a little harder to handle the second part

With this programming model, the service message Req part is passed into the queue as-is
The GeoCom thread does whatever, then populates the Resp part through the notify() syntax
Meanwhile, the ROS service caller threadis blocked until the notify() is called, at which point 
the populated resp is returned to the service caller

This plumbing is needed to ensure that only one thread makes GeoCom calls 
to prevent corrupting the serial stream with interleaved protocol

'''

class MarshalledCall(object):
    def __init__(self, name, req):
        self.cv = Condition()
        self.req = req
        self.res = None
        self.name = name
        self.cv.acquire()
        
    def wait(self):
        self.cv.wait()
        self.cv.release() #since we own the lock after wait() returns
        
    def notify(self, res):
        self.cv.acquire()
        self.res = res
        self.cv.notify() #awake the wait()ing service call
        self.cv.release() #let the service call get the lock


class LeicaNode(object):
    def __init__(self):
        #Pull in params from rosparam
        #Serial port stuff
        self.portName = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baudRate = rospy.get_param('~baudrate', 115200)
        self.counter = 0 
        self.prismname = 'mini'

        #Prism type
        #self.prismType = rospy.get_param('~prismType', 'mini_lp') #or mini_lp_17.5 || mini_360_30
        self.prismType = rospy.get_param('~prismType', 'mini_lp') #mini_360_30

        #Make a command queue
        self.cmdQ = Queue()

        #Tracker state variables
        self.isTracking = False
        
        #Set up the services
        self.laserSvc = rospy.Service('~set_laser', SetBool, self.svcSetLaser)
        self.trackingSvc = rospy.Service('~start_tracking', StartTracking, self.svcStartTracking)
        self.manualSvc = rospy.Service('~stop_tracking', SetBool, self.svcStopTracking)
        self.xaxisSvc = rospy.Service('~set_xaxis', SetBool, self.svcSetXAxis)

        #The tracking should be set up as an action server, since there are multiple intermediate
        #things that can fail, like not finding the prism....
        self.enableAction = actionlib.SimpleActionServer('EnableTracking', EnableTrackingAction, execute_cb=self.enableTracking, auto_start = False)
        self.enableAction.start()
        
        #Set up publishers
        self.statePub = rospy.Publisher('~state', InstrumentState, queue_size=10)
        self.anglePub = rospy.Publisher('~angles', AngleMeasStamped, queue_size=10)
        self.radPub = rospy.Publisher('~radius', RadialMeasStamped, queue_size=10)
        self.cartPub = rospy.Publisher('~position',PointStamped, queue_size=10)

        #Set up cmd_vel subscriber so we can drive the thing remotely via joystick
        self.motorSub = rospy.Subscriber('~cmd_vel', Twist, self.motorCallback)


    def enableTracking(self, goal):
        #Actionlib interface for enabling tracking
        #The action publications are split between the callback thread (this one) and the GeoCom thread
        mc = MarshalledCall('EnableTrackingAction', goal)
        self.cmdQ.put(mc)
        mc.wait()
        self.enableAction.set_succeeded(mc.res)
        return mc.res
       
    def motorCallback(self, cmd):
        #Marshal the motor callback to the Geocom thread for execution
        mc = MarshalledCall('MotorVelocity', cmd)

        #Only queue up a cmd if we can execute it immediately
        if self.cmdQ.empty():
            self.cmdQ.put(mc)
            #No need to wait / notify for completion of motor cmds
        return
    
    def svcStopTracking(self, req):
        # if(self.counter>2):
        #     self.prismType = rospy.get_param('~prismType', 'mini')
        #     print("###############",self.counter)
        #     self.counter = self.counter +1
        # elif(self.counter>5):
        #     #self.prismType = rospy.get_param('~prismType', 'mini_lp')
        #     print('Limit Prism Count Reached')
        #     self.counter = 0
        mc = MarshalledCall('StopTracking', req)
        self.cmdQ.put(mc)
        mc.wait()
        return mc.res
            
    def svcStartTracking(self, req):
        #Make a MarshalledCall to push to the geocom thread:
        marshCall = MarshalledCall('StartTracking', req)
        #The ROS service thread owns the lock, push to the geocom thread's queue
        #The geocom thread may be momentarily paused waiting to acquire the lock currently held by the service thread
        self.cmdQ.put(marshCall)
        #print 'Enqueued cmd from thread ', current_thread().name
        
        #Release the lock and await action by the geocom thread
        marshCall.wait()

        #Geocom thread took action, added a response, and notify()ed us.
        #Return the provided response to the service caller
        return marshCall.res

    def svcSetLaser(self, req):
        marshCall = MarshalledCall('SetLaser', req)
        self.cmdQ.put(marshCall)
        marshCall.wait()
        return marshCall.res

    def svcSetXAxis(self, req):
        marshCall = MarshalledCall('SetXAxis', req)
        self.cmdQ.put(marshCall)
        marshCall.wait()
        return marshCall.res

    def handleStopTracking(self, cmd):
        res = SetBoolResponse()
        #Zeroth, stop tracking if we're already doing that:
        res.success = False

        #Shut down the motor controller
        [error, RC, coord] = GeoCom.MOT_StopController()

        [error, RC, coord] = GeoCom.AUS_SetUserAtrState(0)
        
        [error, RC, coord] = GeoCom.AUS_SetUserLockState(0)
        if RC != 0:
            res.message = str(RC)
            return res
        
        if(self.counter>2):
            self.prismType = rospy.get_param('~prismType', 'mini')
            self.prismType = 'mini'
            print("###############",self.counter)
        if(self.counter>5):
            print('Limit Prism Count Reached')
            print('Setting prism type back to mini_17.5_gate_prism')
            self.prismType = rospy.get_param('~prismType', 'mini_lp')
            self.prismType = 'mini_lp'
            self.counter =0
        
        #Enable motor drive mode
        [error, RC, coord] = GeoCom.MOT_StartController(GeoCom.MOT_CONTROLLER_MODE['MOT_OCONST'])
        if RC != 0:
            res.message = str(RC)
            return res
        
        [error, RC, coord] = GeoCom.TMC_DoMeasure(0,0)
        if RC != 0:
            res.message = str(RC)
            return res

        [error, RC, args] = GeoCom.EDM_Laserpointer(1)
        #Update the tracking state data
        self.handleUpdateState(None)
        res.success = True
        res.message = 'O'
        return res
        
    def handleTrackingAction(self, cmd):
        req = cmd.req #of type EnableTrackingGoal
        feedback = EnableTrackingFeedback()
        
        #Zeroth, stop tracking if we're already doing that:
        [error, RC, coord] = GeoCom.AUS_SetUserLockState(0)
        print('Lock(0), RC:', RC)
        
        [error, RC, coord] = GeoCom.AUS_SetUserAtrState(0)
        print('ATR(0), RC:', RC)
        
        #Update the tracking state data
        self.handleUpdateState(None)
        feedback.status = 'Tracking disabled'
        feedback.rc = 0
        self.enableAction.publish_feedback(feedback)

        #First, set the prism type:
        prism_num = 0
        if self.prismType == 'big':
            prism_num = 3 #big 360 prism
        elif self.prismType == 'mini':
            prism_num = 7 #mini 360 prism
        elif self.prismType == 'mini_lp':
            prism_num = 1 #mini 17.5
        else:
            print('Unknown prism type:', self.prismType)
            return res
        #print('Setting prism num:', prism_num)
        if (prism_num == 7):
            [error, RC, args] = GeoCom.BAP_SetPrismType(prism_num)
        elif(prism_num == 1):
            [error, RC, args] = GeoCom.BAP_SetPrismType2(prism_num) 
        feedback.status = 'Prism set to:', prism_num
        feedback.rc = RC
        self.enableAction.publish_feedback(feedback)

        #Search for the prism:
        Hz = 20
        V = 20

        if GeoCom.AUT_Search(math.radians(Hz),math.radians(V))[1] == 0:
            [error, RC, parameters] = GeoCom.AUT_FineAdjust(math.radians(old_div(Hz,2)),math.radians(old_div(V,2)))
            if RC != 0:
                print('Unable to find prism, code:', RC)
                feedback.status = 'Unable to find prism'
                feedback.rc = RC
                self.enableAction.publish_feedback(feedback)
                self.enableAction.set_aborted()
            
        
        
    def handleStartTracking(self, cmd):
        #print 'Issuing start tracking sequence on thread ', current_thread().name
        res = StartTrackingResponse()
        if self.prismType == 'big':
            prism_num = 3 #big 360 prism
        elif self.prismType == 'mini':
            prism_num = 7 #mini 360 prism
        elif self.prismType == 'mini_lp':
            prism_num = 1 #mini 17.5
        #Zeroth, stop tracking if we're already doing that:
        [error, RC, coord] = GeoCom.AUS_SetUserLockState(0)
        print('Lock(0), RC:', RC)
        
        [error, RC, coord] = GeoCom.AUS_SetUserAtrState(0)
        print('ATR(0), RC:', RC)
        
        #Update the tracking state data
        self.handleUpdateState(None)
        
        #First, set the prism type:
        prism_num = 0
        if self.prismType == 'big':
            prism_num = 3 #big 360 prism
        elif self.prismType == 'mini':
            prism_num = 7 #mini 360 prism
        elif self.prismType == 'mini_lp':
            prism_num = 1
        else:
            print('Unknown prism type:', self.prismType)
            return res
        print('Setting prism num:', prism_num)
        print(self.prismType)
        print(self.counter)
        if(self.counter>2):
            self.prismType = rospy.get_param('~prismType', 'mini')
            self.prismType = 'mini'
            print("###############",self.counter)
        if(self.counter>5):
            print('Limit Prism Count Reached')
            print('Setting prism type back to mini_17.5_gate')
            self.counter = 0
        self.counter = self.counter +1
        [error, RC, parameter] = GeoCom.BAP_GetPrismType()
        print(parameter)

        [error, RC, args] = GeoCom.BAP_SetPrismType(prism_num)
        
        #Search for the prism:
        Hz = 5
        V = 5

        [error, RC, parameters] = GeoCom.AUT_Search(math.radians(Hz),math.radians(V), t_timeout = 10)
        if RC != 0:
            print('Unable to find prism, code:', RC)
            res.rc = RC
            return res
        
        timeoutPeriods = 0
        while timeoutPeriods < 5:
            [error, RC, parameters] = GeoCom.AUT_FineAdjust(math.radians(old_div(Hz,2)),math.radians(old_div(V,2)))
            if RC == 3077:
                timeoutPeriods += 1
                continue
            else:
                break
            
        if RC != 0:
                res.rc = RC
                print('Unable to fine adjust to prism, code:', RC)
                [error, RC, coord] = GeoCom.AUS_SetUserLockState(0)
                print('Lock(0), RC:', RC)
                [error, RC, coord] = GeoCom.AUS_SetUserAtrState(0)
                return res
            
        print('Prism found, enabling ATR')
        [error, RC, coord] = GeoCom.AUS_SetUserLockState(1)
        [error, RC, coord] = GeoCom.AUT_LockIn()
        if RC != 0:
            print('Unable to lock prism')
            res.rc = RC
            return res

        #Set fast measurements
        GeoCom.TMC_SetEdmMode(9)

        #Start the measurement module
        GeoCom.TMC_DoMeasure()
        rospy.sleep(1)
        
        return res
    
    def handleSetLaser(self, cmd):
        res = SetBoolResponse()
        if cmd.req.data == True:
            value = 1
        else:
            value = 0
            
        [error, RC, args] = GeoCom.EDM_Laserpointer(value)
        '''
        if value == 1:
            [error, RC, args] = GeoCom.EDM_SetEGLIntensity(GeoCom.EDM_INTENSITY['EDM_EGLINTEN_HIGH'])
        else:
            [error, RC, args] = GeoCom.EDM_SetEGLIntensity(GeoCom.EDM_INTENSITY['EDM_EGLINTEN_OFF'])
        '''        
        if error == 0:
            res.success = True
        else:
            res.success = False

        res.message = str(RC)
        return res

    def handleUpdateState(self, cmd):
        power_state = GeoCom.CSV_CheckPower()[2]
        #print 'Power state:', power_state

        lock_state = GeoCom.MOT_ReadLockStatus()[2]
        #print 'Lock status:', lock_state

        #Potentially include the UserLock and UserATR state variables..
        
        msg = InstrumentState()
        msg.header.stamp = rospy.Time.now()
        msg.lock_status = GeoCom.MOT_LOCK_STATUS[int(lock_state)]
        msg.battery_pct = int(power_state[0])
        msg.power_source = GeoCom.CSV_POWER_SOURCE[int(power_state[1])]
        self.statePub.publish(msg)

        #Update internal state variables 
        if msg.lock_status == GeoCom.MOT_LOCK_STATUS[1]:
            self.isTracking = True
        else:
            self.isTracking = False
            #Enable the laser automatically
            GeoCom.EDM_Laserpointer(1)
        return None

    def handleSetXAxis(self, cmd):
        #Set cartesian coordinates, station is at 000 on the x-axis
        [error, RC, args] = GeoCom.TMC_SetOrientation()
        res = SetBoolResponse()
        if error == 0:
            res.success = True
        else:
            res.success = False

        res.message = str(RC)
        return res
    
    def handleMotorVelocity(self, cmd):
        #print("handling motor vel")
        #Check to see if we're locked in. Only attempt to drive the motors if we're unlocked
        if self.isTracking:
            #print("canceling bc tracking")
            return
       
        #Pull the requested x/y vels from the Twist msg:
        h_vel = cmd.req.angular.z
        v_vel = cmd.req.angular.y
        [error, RC, params] = GeoCom.MOT_SetVelocity(h_vel,v_vel)
        if RC != 0:
            print("RC!=0")
            if RC == 1794:
                #Motor is not in OCONST state, no biggie
                pass
            else:
                print('Motor drive: Got an unusual RC:', RC)
        
            
    def handleService(self, cmd):
        #Ensure we have a MarshalledCall to deal with
        if isinstance(cmd, MarshalledCall):
            #print 'Got a good service object!'
            #Figure out what service to call
            if cmd.name == 'StartTracking':
                cmd.notify(self.handleStartTracking(cmd))
            elif cmd.name == 'StopTracking':
                cmd.notify(self.handleStopTracking(cmd))
            elif cmd.name == 'EnableTrackingAction':
                cmd.notify(self.handleTrackingAction(cmd))
            elif cmd.name == 'SetLaser':
                cmd.notify(self.handleSetLaser(cmd))
            elif cmd.name == 'UpdateState':
                cmd.notify(self.handleUpdateState(cmd))
            elif cmd.name == 'SetXAxis':
                cmd.notify(self.handleSetXAxis(cmd))
            elif cmd.name == 'MotorVelocity':
                self.handleMotorVelocity(cmd)
            else:
                print(('Unknown service request:', cmd.name))
        else:
            print(('Unknown enqueued work item class:', type(cmd)))
        return
    
    def updateState(self, event):
        #rospy.Timer callbacks don't execute on the main thread...
        #Marshal to the main cmd q for execution
        marshCall = MarshalledCall('UpdateState', event)
        self.cmdQ.put(marshCall)
        marshCall.wait()
        #We publish from the main thread, where we can update internal state monitoring of the class
        return None


    def publishMeasurement(self):
        #print 'Publishing measurement'
        [error, RC, coord] = GeoCom.TMC_GetSimpleMea(5, 1) #TMC_AUTO_INC mode, TMC_INCLINE_PRG
        if RC != 0:
            print('Measurement anomaly:', RC)
            return
        
        phi = -float(coord[0])
        theta = float(coord[1])
        radius = float(coord[2])

        radius_meas = RadialMeasStamped()
        angle_meas = AngleMeasStamped()
        radius_meas.header.stamp = rospy.Time.now()
        angle_meas.header.stamp = radius_meas.header.stamp

        radius_meas.radius = radius
        angle_meas.phi = phi
        angle_meas.theta = theta
        self.anglePub.publish(angle_meas)
        self.radPub.publish(radius_meas)

        #Compute cartesian measurement and publish
        cart_meas = PointStamped()
        cart_meas.header.stamp = rospy.Time.now()
        cart_coords = self.getCartesian(phi, theta, radius)
        
        cart_meas.point.x = cart_coords[0]
        cart_meas.point.y = cart_coords[1]
        cart_meas.point.z = cart_coords[2]
        self.cartPub.publish(cart_meas)
        
        #rospy.sleep(0.1)
        
    def getCartesian(self, phi, theta, radius):
        """
        Compute cartesian coordinates using vertical, horizontal angles and distance measurements.
        :param phi: horizontal angle (rad)
        :type phi: float
        :param theta: vertical angle (rad)
        :type theta: float
        :param radius: distance from the station to the prism (m)
        :type radius: float
        :returns: a tuple with coordinates [x,y,z]
        """
        point_x = round(sin(theta) * cos(phi) * radius,4)
        point_y = round(sin(theta) * sin(phi) * radius,4)
        point_z = round(cos(theta) * radius,4)
        return [point_x, point_y, point_z]

    def run(self):
        '''
        Main loop: 
          Check the command queue - if there's something there, do that
           Ring the bell when it completes
          Pull status
          If we're in tracking, pull a measurement and publish it
        '''
        #Open the connection to the TS:
        print('Using port:', self.portName)

        connection = GeoCom.COM_OpenConnection(self.portName, self.baudRate)
        if connection[0]:
            print('Unable to open connection to port:', self.portName)
            return

        #Find the particulars of the connected TS:
        inst_name = GeoCom.CSV_GetInstrumentName()[2]
        serial_num = GeoCom.CSV_GetInstrumentNo()[2]
        print('Found TS:',  inst_name, ' with instrument number:', serial_num)
        #Initialize the TS to a known state:
        self.handleStopTracking(None)
        
        #Limit state publishing to 2 Hz so as to not soak up serial bandwidth
        self.state_timer = rospy.Timer(rospy.Duration(0.5), self.updateState)
        
        #Start handling service calls and potentially making measurements:
        while not rospy.is_shutdown():
            if not self.cmdQ.empty():
                #get something off of the command queue
                newCmd = self.cmdQ.get()
                #process the command
                self.handleService(newCmd)

            if self.isTracking: #We have weapons lock!
                self.publishMeasurement()

def main():
    rospy.init_node('leica_node')
    node = LeicaNode()
    node.run()
    
if __name__ == '__main__':
    main()
