#!/usr/bin/python2
"""
Leica UI node to monitor the prism data
"""

import sys
import rospy 
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
# 
import math
import tf
import numpy as np  
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
import itertools as it
import time
# from QLabeledValue import *
from leica_ros.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *

from leica_ros.srv import *
import message_filters
# from std_msgs.srv import *

from ltf.srv import gettf
from ltf.srv import gettfRequest
from ltf.srv import gettfResponse

def multiply_tfs(trans1, rot1, trans2, rot2):

    trans1_mat = tf.transformations.translation_matrix(trans1)
    rot1_mat   = tf.transformations.quaternion_matrix(rot1)
    mat1 = np.dot(trans1_mat, rot1_mat)

    trans2_mat = tf.transformations.translation_matrix(trans2)
    rot2_mat    = tf.transformations.quaternion_matrix(rot2)
    mat2 = np.dot(trans2_mat, rot2_mat)

    mat3 = np.dot(mat1, mat2)
    trans3 = tf.transformations.translation_from_matrix(mat3)
    rot3 = tf.transformations.quaternion_from_matrix(mat3)

    return trans3, rot3

class prism_tf:
    def __init__(self):
        ang = np.array([0,0,0,1])
        pos = np.array([0,0,0])
P1 = prism_tf()
P2 = prism_tf()
P3 = prism_tf()
P4 = prism_tf()
P5 = prism_tf()
P6 = prism_tf()
world = [prism_tf() for _ in range(7)]
w = []
q = []
world_origin_pos = []
world_origin_rot = []
count =1 
c=1

class PrismMonitorWidget(QWidget): 
    commandUpdate = pyqtSignal(Transform)
    Current_P = prism_tf()
    counter = 0

    def __init__(self,parent = None):
        self.counter = 0
        self.Current_P = prism_tf()

        super(PrismMonitorWidget,self).__init__()
        layout = QVBoxLayout()
        #Prism 1
        self.counter =1
        self.check = True
        prismLayout = QVBoxLayout()
        self.controlGroup = QGroupBox('Prism Left Lower Gate') 
        self.prismG1 = QPushButton('Calculate')
        self.prismG1.clicked.connect(self.prismG1_onclick) 
        prismLayout.addWidget(self.prismG1)
        self.controlGroup.setLayout(prismLayout)
        layout.addWidget(self.controlGroup)

        #Prism 2
        self.counter =2
        prismLayout = QVBoxLayout()
        self.controlGroup = QGroupBox('Prism Top Gate') 
        self.prismG2 = QPushButton('Calculate') 
        self.prismG2.clicked.connect(self.prismG2_onclick) 
        prismLayout.addWidget(self.prismG2)
        self.controlGroup.setLayout(prismLayout)
        layout.addWidget(self.controlGroup)
        
        #Prism 3
        self.counter =3
        prismLayout = QVBoxLayout()
        self.controlGroup = QGroupBox('Prism Right Lower Gate') 
        self.prismG3 = QPushButton('Calculate') 
        self.prismG3.clicked.connect(self.prismG3_onclick) 
        prismLayout.addWidget(self.prismG3)
        self.controlGroup.setLayout(prismLayout)
        layout.addWidget(self.controlGroup)
        
        #Prism 4
        self.counter =4
        prismLayout = QVBoxLayout()
        self.controlGroup = QGroupBox('Prism Left Robot') 
        self.prismR1 = QPushButton('Calculate') 
        self.prismR1.clicked.connect(self.prismR1_onclick) 
        prismLayout.addWidget(self.prismR1)
        self.controlGroup.setLayout(prismLayout)
        layout.addWidget(self.controlGroup)
        
        #Prism 5
        self.counter =5
        prismLayout = QVBoxLayout()
        self.controlGroup = QGroupBox('Prism Top Robot') 
        self.prismR2 = QPushButton('Calculate') 
        self.prismR2.clicked.connect(self.prismR2_onclick) 
        prismLayout.addWidget(self.prismR2)
        self.controlGroup.setLayout(prismLayout)
        layout.addWidget(self.controlGroup)

        #Prism 6
        self.counter =6
        prismLayout = QVBoxLayout()
        self.controlGroup = QGroupBox('Prism Right Robot') 
        self.prismR3 = QPushButton('Calculate') 
        self.prismR3.clicked.connect(self.prismR3_onclick) 
        prismLayout.addWidget(self.prismR3)
        self.controlGroup.setLayout(prismLayout)
        layout.addWidget(self.controlGroup)
       
        #Exit Button layout
        self.btnQuit = QPushButton('Exit')
        self.btnQuit.clicked.connect(self.btnQuit_onclick)
        layout.addWidget(self.btnQuit)

        self.setLayout(layout)

    def call_listener(self,pos,ang):
        try:
            global count 
            global world_origin_pos
            global w    
            #Looks Up the static transform from the world_origin_pos to the
            tf_listener = tf.TransformListener()
            if(count==1):
                tf_listener.waitForTransform("/Gate_Origin", "/PrismLeftLowerGate", rospy.Time(),  rospy.Duration(2))
                (prsm_pose, prsm_rot) = tf_listener.lookupTransform("/Gate_Origin","/PrismLeftLowerGate", rospy.Time(0))
            if(count==2):
                tf_listener.waitForTransform("/Gate_Origin", "/PrismTopGate", rospy.Time(),  rospy.Duration(2))
                (prsm_pose, prsm_rot) = tf_listener.lookupTransform("/Gate_Origin","/PrismTopGate", rospy.Time(0))
            if(count==3):
                tf_listener.waitForTransform("/Gate_Origin", "/PrismRightLowerGate", rospy.Time(),  rospy.Duration(2))
                (prsm_pose, prsm_rot) = tf_listener.lookupTransform("/Gate_Origin","/PrismRightLowerGate", rospy.Time(0))
            if(count==4):
                tf_listener.waitForTransform("/Robot_Origin", "/RobotLeftPrism", rospy.Time(),  rospy.Duration(2))
                (prsm_pose, prsm_rot) = tf_listener.lookupTransform("/Robot_Origin","/RobotLeftPrism", rospy.Time(0))
            if(count==5):
                tf_listener.waitForTransform("/Robot_Origin", "/RobotTopPrism", rospy.Time(),  rospy.Duration(2))
                (prsm_pose, prsm_rot) = tf_listener.lookupTransform("/Robot_Origin","/RobotTopPrism", rospy.Time(0))
            if(count==6):
                tf_listener.waitForTransform("/Robot_Origin", "/RobotRightPrism", rospy.Time(),  rospy.Duration(2))
                (prsm_pose, prsm_rot) = tf_listener.lookupTransform("/Robot_Origin","/RobotRightPrism", rospy.Time(0))

            transform = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(prsm_pose), tf.transformations.quaternion_matrix(prsm_rot))
            inversed_tf = tf.transformations.inverse_matrix(transform)
            trans1 = tf.transformations.translation_from_matrix(inversed_tf)
            rot1 = tf.transformations.quaternion_from_matrix(inversed_tf)                
            trans_multiplied, rot_multiplied = multiply_tfs(pos,ang, trans1, rot1)

            w.append(trans_multiplied)
            q.append(rot_multiplied)
            
            if(count==3):
                world_origin_pos.append(np.mean(w[0:3],axis=0))
                world_origin_rot.append(np.mean(q[0:3],axis=0))
                # print("Position of World Origin Distance With respect to Leica_Total_Station")
                # print(world_origin_pos)
                # # print("Position of World Origin Rotation With respect to Leica_Total_Station")
                # # print(world_origin_rot)
                to_show_world_origin_pos = tf.transformations.translation_matrix(world_origin_pos[0].tolist())
                print("Position of World Origin Distance With respect to Leica_Total_Station")
                print(to_show_world_origin_pos)
                to_show_world_origin_rot  = tf.transformations.quaternion_matrix(world_origin_rot[0].tolist())
                print("Position of World Origin Rotation With respect to Leica_Total_Stationl")
                print(to_show_world_origin_rot)
                # mat1 = np.dot(to_show_world_origin_pos, to_show_world_origin_rot )
                # print("Orientation of the world with Respect to Leica Total Statio is")
                # print(mat1)
            elif(count==6):
                world_origin_pos.append(np.mean(w[3:],axis=0))
                world_origin_rot.append(np.mean(q[3:],axis=0))
                # print("Position of Robot Origin with respect to Leica_Total_Station")
                # print(world_origin_pos[1])
                # print("Position of Robot Origin with respect to Leica_Total_Station")
                # print(world_origin_rot[1])
                print("*****************************************************")
                to_show_world_origin_pos_2 = tf.transformations.translation_matrix(world_origin_pos[1].tolist())
                print("Position of Robot Origin Distance With respect to Leica_Total_Station")
                print(to_show_world_origin_pos_2)
                to_show_world_origin_rot_2 = tf.transformations.quaternion_matrix(world_origin_rot[1].tolist())
                print("Position of Robot Origin Rotation With respect to Leica_Total_Stationl")
                print(to_show_world_origin_rot_2)
                print("*****************************************************")
                print("\n")
                 
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            detection_counter = 0
            #continue

    def prismG1_onclick(self):
        #Make the service call to enable the robot
        rospy.wait_for_service('prismTransform')
        try: 
            rospy.loginfo('\nPrism:transformations from leica_home are')
            prismTransform = rospy.ServiceProxy('prismTransform', gettf) # s = client object
            req = gettfRequest() 
            resp = prismTransform(req)
            #if(self)
            self.Current_P.ang=resp.ang
            self.Current_P.ang=resp.pos 
            global count
            if(count==1):
                P1.ang = resp.ang
                P1.pos = resp.pos
                print("Prism no.:",count)
                print("***********Prism Position***********")
                print(P1.pos)
                print("***********************************************")
                self.call_listener(P1.pos,P1.ang)
                print('\n')
            if(count<=6):
                count = count +1
            self.prismG1.setEnabled(False)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def prismG2_onclick(self):
            
            #Make the service call to enable the robot
            rospy.wait_for_service('prismTransform')
            try: 
                # rospy.loginfo('\nPrism:transformations from leica_home are')
                prismTransform = rospy.ServiceProxy('prismTransform', gettf) # s = client object
                req = gettfRequest() 
                resp = prismTransform(req)
                #if(self)
                self.Current_P.ang=resp.ang
                self.Current_P.ang=resp.pos 
                global count
                if(count==2):
                    P2.ang = resp.ang
                    P2.pos = resp.pos
                    print("Prism no.:",count)
                    print("***********Prism Position***********")
                    print(P2.pos)
                    print("***********************************************")
                    self.call_listener(P2.pos,P2.ang)
                    print('\n')
                if(count<=6):
                    count = count +1
                self.prismG2.setEnabled(False)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

    def prismG3_onclick(self):
            
            #Make the service call to enable the robot
            rospy.wait_for_service('prismTransform')
            try: 
                # rospy.loginfo('\nPrism:transformations from leica_home are')
                prismTransform = rospy.ServiceProxy('prismTransform', gettf) # s = client object
                req = gettfRequest() 
                resp = prismTransform(req)
                #if(self)
                self.Current_P.ang=resp.ang
                self.Current_P.ang=resp.pos 
                global count
                if(count==3):
                    P3.ang = resp.ang
                    P3.pos = resp.pos
                    print("Prism no.:",count)
                    print("***********Prism Position***********")
                    print(P3.pos)
                    print("***********************************************")
                    self.call_listener(P3.pos,P3.ang)
                    print('\n')
                    
                if(count<=6):
                    count = count +1
                self.prismG3.setEnabled(False)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

    def prismR1_onclick(self):
            
            #Make the service call to enable the robot
            rospy.wait_for_service('prismTransform')
            try: 
                # rospy.loginfo('\nPrism:transformations from leica_home are')
                prismTransform = rospy.ServiceProxy('prismTransform', gettf) # s = client object
                req = gettfRequest() 
                resp = prismTransform(req)
                #if(self)
                self.Current_P.ang=resp.ang
                self.Current_P.ang=resp.pos 
                global count
                if(count==4):
                    P4.ang = resp.ang
                    P4.pos = resp.pos
                    print("Prism no.:",count)
                    print("***********Prism Position***********")
                    print(P4.pos)
                    print("***********************************************")
                    self.call_listener(P4.pos,P4.ang)
                    print('\n')
                if(count<=6):
                    count = count +1
                self.prismR1.setEnabled(False)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

    def prismR2_onclick(self):
            
            #Make the service call to enable the robot
            rospy.wait_for_service('prismTransform')
            try: 
                # rospy.loginfo('\nPrism:transformations from leica_home are')
                prismTransform = rospy.ServiceProxy('prismTransform', gettf) # s = client object
                req = gettfRequest() 
                resp = prismTransform(req)
                #if(self)
                self.Current_P.ang=resp.ang
                self.Current_P.ang=resp.pos 
                global count
                if(count==5):
                    P5.ang = resp.ang
                    P5.pos = resp.pos
                    print("Prism no.:",count)
                    print("***********Prism Position***********")
                    print(P5.pos)
                    print("***********************************************")
                    self.call_listener(P5.pos,P5.ang)
                    print('\n')
                if(count<=6):
                    count = count +1
                self.prismR2.setEnabled(False)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

    def prismR3_onclick(self):
            
            #Make the service call to enable the robot
            rospy.wait_for_service('prismTransform')
            try: 
                # rospy.loginfo('\nPrism:transformations from leica_home are')
                prismTransform = rospy.ServiceProxy('prismTransform', gettf) # s = client object
                req = gettfRequest() 
                resp = prismTransform(req)
                #if(self)
                self.Current_P.ang=resp.ang
                self.Current_P.ang=resp.pos 
                global count
                if(count==6):
                    P6.ang = resp.ang
                    P6.pos = resp.pos
                    print("Prism no.:",count)
                    print("***********Prism Position***********")
                    print(P6.pos)
                    print("***********************************************")
                    self.call_listener(P6.pos,P6.ang)
                    print('\n')
                    
                if(count<=6):
                    count = count +1
                self.prismR3.setEnabled(False)
                if(count>6):
                    print("Prism Count limit reached")
                    count = 0        
                    self.World_Robot_Origin()
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

    def btnQuit_onclick(self):
        self.parent().close()

    def World_Robot_Origin(self):        
        transform = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(world_origin_pos[1]), tf.transformations.quaternion_matrix(world_origin_rot[1]))
        inversed_tf = tf.transformations.inverse_matrix(transform)
        trans_inv_1 = tf.transformations.translation_from_matrix(inversed_tf)
        rot_inv_1 = tf.transformations.quaternion_from_matrix(inversed_tf)  
        trans, rot = multiply_tfs(world_origin_pos[0], world_origin_rot[0], trans_inv_1, rot_inv_1)
        print("Distance of Origin from ")
        print(trans)
        orientation = tf.transformations.euler_from_quaternion(rot)
        print("Orientation")
        print(orientation)
        tv_mat = tf.transformations.translation_matrix(trans)
        rv_mat   = tf.transformations.quaternion_matrix(rot)
        mat = np.dot(tv_mat, rv_mat)

        print("Transformation matrix is")
        print(mat)




def main():

    rospy.init_node('prism_monitor_node')
    app = QApplication(sys.argv)
    mainWidget = PrismMonitorWidget(app)
    mainWindow = QMainWindow()
    mainWindow.setWindowTitle('Prism Position Tracker')
    mainWindow.setCentralWidget(mainWidget)
    mainWindow.setStatusBar(QStatusBar())
    mainWindow.show()
    app.exec_()
    

if __name__ == '__main__':
    try:
	main()
    except rospy.ROSInterruptException:
	pass










