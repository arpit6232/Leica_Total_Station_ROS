#!/usr/bin/env python2
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
from ltf.srv import gettf
from ltf.srv import gettfRequest
from ltf.srv import gettfResponse

def normalize_quaternion(rot):
    ratio = math.sqrt(rot[0]**2 + rot[1]**2 + rot[2]**2 + rot[3]**2)
    return (rot[0]/ratio, rot[1]/ratio, rot[2]/ratio, rot[3]/ratio)

# Find mode in a series of observations
def get_mode(arr, tolerance):
    arr.sort()
    mode = arr[0]
    mode_count = 0

    curr_count = 0
    curr_element = mode
    for x in arr:
        if abs(x - curr_element) <= tolerance:
            curr_count = curr_count + 1
            if curr_element == mode:
                mode_count = mode_count + 1
        else:
            curr_element = x
            curr_count = 1

        if curr_count > mode_count:
            mode = curr_element
            mode_count = curr_count
    return mode

def filter(samples, tolerance):
    idx = filter_idx(samples, tolerance)
    return np.mean(samples[idx])

def filter_sub(subsamples):
    return np.mean(subsamples)

def filter_idx(samples, tolerance):
    samples_temp = np.copy(samples)
    mode = get_mode(samples_temp, tolerance)
    diff = abs(samples - mode)
    idx = np.where(diff <= tolerance)
    return idx[0]

def is_stable(samples, idx):
    ratio = idx.shape[0]/float(len(samples))
    return ratio > 0.6

class findPrismLoc:
    def __init__(self):
        self.enable_prism = True
        self.enable_stop_for_prism = True

        self.leica_home = "home"
        self.prism_frame = "prism"

        self.prism_found = False


        #Spinning rate for rospy
        self.rate=100

        #number of self.num_filter_frames to run filterning over
        self.num_filter_frames = 25

        # No of self.num_filter_frames required to detect whether the prism data has been found
        self.num_stable_prism_frames = 3

        # Paramter for filtering scheme
        self.filter_tol = 0.0001

        # Closeness of new detections be to old ones
        self.consec_dist_tol = 0.00005

        self.final_pos = (0,0,0)
        self.final_rot = (0,0,0,1)

        self.prism_pose = (0,0,0)
        self.prism_rot = (0,0,0,1)

        self.tf_listener = tf.TransformListener()
        self.tf_caster = tf.TransformBroadcaster()
    
    
    def findPrism(self):

        px_prism = [0] * self.num_filter_frames
        py_prism = [0] * self.num_filter_frames
        pz_prism = [0] * self.num_filter_frames

        # Arrays to store prism detections
        detection_counter = 0

        while not self.prism_found:    

            
            try:
                self.tf_listener.waitForTransform(self.leica_home, self.prism_frame, rospy.Time.now(),  rospy.Duration(2))
                (prsm_pose, prsm_rot) = self.tf_listener.lookupTransform(self.leica_home, self.prism_frame, rospy.Time(0))
                self.prism_pose = prsm_pose
                self.prism_rot = prsm_rot
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                #rospy.loginfo_throttle(1,"No prism tf available.")
                #print(e)
                detection_counter = 0
                continue
            

            # Add pose to arrays
            px_prism[detection_counter % self.num_filter_frames] = self.prism_pose[0]
            py_prism[detection_counter % self.num_filter_frames] = self.prism_pose[1]
            pz_prism[detection_counter % self.num_filter_frames] = self.prism_pose[2]

            detection_counter = detection_counter + 1
            
        # Filtering position
            if detection_counter >= self.num_filter_frames:
                # # rospy.loginfo("Prism counter reached.")
                px_idx = filter_idx(np.array(px_prism), self.filter_tol)
                py_idx = filter_idx(np.array(py_prism), self.filter_tol)
                pz_idx = filter_idx(np.array(pz_prism), self.filter_tol)

                tf_is_stable = is_stable(px_prism,px_idx) and \
                    is_stable(py_prism,py_idx) and \
                    is_stable(pz_prism,pz_idx)

                if tf_is_stable:
                    rospy.loginfo("Prism stable.")
                
                    stable_prism_pose = (filter_sub(np.array(px_prism)[px_idx]), filter_sub(np.array(py_prism)[py_idx]), filter_sub(np.array(pz_prism)[pz_idx]))
                    self.final_pos = stable_prism_pose
                    self.final_rot = self.prism_rot
                    self.final_rot = normalize_quaternion(self.prism_rot)

                    self.prism_found = True
                    rospy.loginfo("Prism found.")
                    #print(self.final_pos,Rotation.from_quat(self.final_rot).as_euler('xyz', degrees=True))
                    #return gettfResponse(self.final_rot,self.final_pos)
                    #print("##################################################")
            rospy.Rate(self.rate).sleep()

def callback(req):
    P = findPrismLoc()
    P.findPrism() 
    return gettfResponse(P.final_rot,P.final_pos)

def prismtransform_Server():
    rospy.init_node('prismTransform')
    s = rospy.Service('prismTransform',gettf,callback)

if __name__ == "__main__":
     prismtransform_Server()
     rospy.spin()