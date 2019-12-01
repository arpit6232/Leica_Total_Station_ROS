#!/usr/bin/env python
import rospy
import roslib
#roslib.load_manifest('ltf')
import math
import tf
from leica_ros.msg import AngleMeasStamped
from  geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
import numpy as np  
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
import message_filters
import time


def callback(angCallback, posCallback):
    #Creating a transform broadcaster
    transform_broadcaster  = tf.TransformBroadcaster() 
    #rate = rospy.Rate(1.0)
    #Converting the Azimuth and Zenith angle's leica to Euler - roll,pitch,yaw
    roll = 0
    pitch = math.radians(angCallback.theta)
    yaw = math.radians(angCallback.phi)
    
    #Setting the translation vector 
    translation_vector = (posCallback.point.x,posCallback.point.y,posCallback.point.z) 
  
    #roll-pitch-yaw angles to a quaternion using ROS TF Library
    rotation_quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    #Capturing current time 
    current_time = rospy.Time.now()

    #Broadcasting frame from home to 1st Prism
    transform_broadcaster.sendTransform(translation_vector,rotation_quaternion,current_time,"home","prism")
    #rate.sleep(1.0)
    

    
def main():
    rospy.init_node('leica_tf_broadcaster', anonymous=True )
    #To Generate the frame from Parent-Home to Child-Prism
    angle_sub = message_filters.Subscriber("/leica_node/angles", AngleMeasStamped)
    dist_sub = message_filters.Subscriber("/leica_node/position", PointStamped)

    ts = message_filters.ApproximateTimeSynchronizer([angle_sub,dist_sub],10,0.1,allow_headerless=True)
    ts.registerCallback(callback) 
    rospy.spin()

if __name__ == '__main__':
    main()
