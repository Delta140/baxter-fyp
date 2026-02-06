#!/usr/bin/env python

#FYP project
#Author Alevi Bingham
import argparse
import rospy
import cv2
import cv_bridge
import baxter_interface
import baxter_external_devices
import numpy as np
import struct
import sys
import math
import threading
import time
#import mediapipe as mp
bridge = cv_bridge.CvBridge()
from sensor_msgs.msg import (
    Image,
    JointState,
)

from std_msgs.msg import Header

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


from baxter_interface import CHECK_VERSION



WindowX=440
WindowY=80
WindowSize=80

FlagLeft=False
FlagRight=False

global objectfound
objectfound=False

xL, yL, wL, hL = 0, 0, 0, 0

xR, yR, wR, hR = 0, 0, 0, 0

lower_threshold = np.array([0, 15, 0], dtype=np.uint8)
upper_threshold = np.array([20, 255, 255], dtype=np.uint8)

def IKSolver(NewPos, NewOri, limb):
    ns = "ExternalTools/"+limb+"/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses={
            limb: PoseStamped(
                header=hdr,
                pose=Pose(
                        position=NewPos,
                        orientation=NewOri
                        )
            )
        }
    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1
        
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                           resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        return limb_joints
        #print("solution found")
    else:
        #print("no solution found")
        return 0

def image_callback_left(ros_img): 
    global objectfound
    global xL, yL, wL, hL
  
    # Convert received image message to OpenCv image 
    cv_image = bridge.imgmsg_to_cv2(ros_img, desired_encoding="passthrough")
    cv2.rectangle(cv_image, (WindowX, WindowY), (WindowX+WindowSize, WindowY+WindowSize), (255, 0, 0), 2)     
    hls_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HLS)
    
    # Define lower and upper threshold values for skin detection in HLS
    
    # Create a binary mask of potential skin regions
    skin_mask = cv2.inRange(hls_image, lower_threshold, upper_threshold)
    
    # Perform morphological operations to refine the mask
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    skin_mask = cv2.morphologyEx(skin_mask, cv2.MORPH_OPEN, kernel, iterations=2)
    skin_mask = cv2.dilate(skin_mask, kernel, iterations=1)


    # Find contours
    contours, _ = cv2.findContours(skin_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Draw largest contour (assumed to be hand)
    if contours:
        max_contour = max(contours, key=cv2.contourArea)
        xL, yL, wL, hL = cv2.boundingRect(max_contour) 
        if cv2.contourArea(max_contour) > 1000:
            cv2.rectangle(cv_image, (xL, yL), (xL + wL, yL + hL), (0, 255, 0), 2)
    #cv2.imshow('Hand Detection left', cv_image) 
    cv2.waitKey(1)

    if(objectfound == False):
        effort=left.endpoint_effort()
        force= effort["force"]
        ForceZ=force.z
        if(ForceZ<-3):
            print("object found")
            objectfound=True

def image_callback_right(ros_img):
    global xR, yR, wR, hR 

    # Convert received image message to OpenCv image 
    cv_image = bridge.imgmsg_to_cv2(ros_img, desired_encoding="passthrough")
    cv2.rectangle(cv_image, (WindowX, WindowY), (WindowX+WindowSize, WindowY+WindowSize), (255, 0, 0), 2)     
    hls_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HLS)
    
    # Define lower and upper threshold values for skin detection in HLS
    
    # Create a binary mask of potential skin regions
    skin_mask = cv2.inRange(hls_image, lower_threshold, upper_threshold)
    
    # Perform morphological operations to refine the mask
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    skin_mask = cv2.morphologyEx(skin_mask, cv2.MORPH_OPEN, kernel, iterations=2)
    skin_mask = cv2.dilate(skin_mask, kernel, iterations=1)


    # Find contours
    contours, _ = cv2.findContours(skin_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Draw largest contour (assumed to be hand)
    if contours:
        max_contour = max(contours, key=cv2.contourArea)
        xR, yR, wR, hR = cv2.boundingRect(max_contour) 
        if cv2.contourArea(max_contour) > 1000:
            cv2.rectangle(cv_image, (xR, yR), (xR + wR, yR + hR), (0, 255, 0), 2)
    cv2.imshow("Hand Detection Right", cv_image)
    cv2.waitKey(1) 

   
        

def RightArmControl():
    while (Shutdown==False):
        print(left.endpoint_effort())
        time.sleep(0.5)
    """
    global FlagRight
    rospy.Subscriber('/cameras/right_hand_camera/image', Image, image_callback_right) 

    #print(diffx)
    #print(diffy)
    while(True):
        diffx= abs(WindowX-xR)*0.0003
        diffy= (abs(WindowY - yR))*0.0003
        Pos = right.endpoint_pose()

        CurrentPos=Pos["position"]
        CurrentOri=Pos["orientation"]
        
        PosX=CurrentPos.x
        #PosY=CurrentPos.y
        PosZ=CurrentPos.z

        if((xR+wR/2)>WindowX+WindowSize):
            CurrentPos=CurrentPos._replace(z=PosZ-diffx)
            FlagRight=False
        elif((xR+wR/2)< WindowX):
            CurrentPos=CurrentPos._replace(z=PosZ+diffx)
            FlagRight=False
        if((yR+hR/2)> WindowY+WindowSize):
            CurrentPos=CurrentPos._replace(x=PosX-diffy)
            FlagRight=False
        elif((yR+hR/2)< WindowY):    
            CurrentPos=CurrentPos._replace(x=PosX+diffy)
            FlagRight=False
        else:
            FlagRight=True
            
        movement=IKSolver(CurrentPos,CurrentOri, 'right')
        if(movement!=0):
            right.move_to_joint_positions(movement)
    """    

    
def LeftArmControl():
    global FlagLeft
    global objectfound
    rospy.Subscriber('/cameras/left_hand_camera/image', Image, image_callback_left)
    while(Shutdown==False):
        diffx=( abs(WindowX-xL))*0.0003
        diffy= (abs(WindowY - yL))*0.0003
        
        Pos = left.endpoint_pose()
        CurrentPos=Pos["position"]
        CurrentOri=Pos["orientation"]
        
        PosX=CurrentPos.x
        PosY=CurrentPos.y
        PosZ=CurrentPos.z

        
        if((xL+wL/2)>WindowX+WindowSize):
            CurrentPos=CurrentPos._replace(y=PosY-diffx)
            FlagLeft=False
        elif((xL+wL/2)< WindowX):
            CurrentPos=CurrentPos._replace(y=PosY+diffx)
            FlagLeft=False
   
        if((yL+hL/2)> WindowY+WindowSize):
            CurrentPos=CurrentPos._replace(x=PosX-diffy)
            FlagLeft=False
        elif((yL+hL/2)< WindowY):    
            CurrentPos=CurrentPos._replace(x=PosX+diffy)
            FlagLeft=False

        if(((yL+hL/2)> WindowY) and((yL+hL/2)< WindowY+WindowSize) and ((xL+wL/2)> WindowX) and ((xL+wL/2)<WindowX+WindowSize)):
            FlagLeft=True

        movement=IKSolver(CurrentPos,CurrentOri, "left")
        #print(movement)
        if(movement !=0):
            left.move_to_joint_positions(movement)

        if(FlagLeft==True and objectfound == False):
            CurrentPos=CurrentPos._replace(z=PosZ-0.05)
            movement=IKSolver(CurrentPos,CurrentOri, "left")
            #print(movement)
            if(movement !=0 and objectfound==False):
                left.move_to_joint_positions(movement)
            FlagLeft==False
        elif(objectfound == True):
              grip_left.close()
              time.sleep(3)
              left.move_to_joint_positions(jointposL)
              grip_left.open()
              left.move_to_joint_positions(jointposL)
              objectfound=False
              FlagLeft=False
            
    
        
    

if __name__ == '__main__':

    print("Initializing node... ")
    rospy.init_node("Project1")
    print("Getting robot state... ")

    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    print("Enabling robot... ")
    rs.enable()

    Shutdown=False

    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_left.open()
    grip_left.calibrate()

    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    lj = left.joint_names()
    rj = right.joint_names()

    templ = left.joint_angles()
    print(templ)
    #tempr = right.joint_angles()
    #print(tempr)
    #s0 s1 e0 e2 w0 w1 w2
    NeutralL ={lj[0]:-0.5798447378206864, lj[1]:-0.7435971869274544, lj[2]:-0.985199161019407, lj[3]:1.7203594536134914, lj[4]:0.8751360394886285,  lj[5]:1.0089758632316308, lj[6]:-1.0216312047316856,}
    jointposL ={lj[0]:-1.1800147210808545, lj[1]:-0.7850146682003605, lj[2]:0.25617479157686407, lj[3]:0.2619272195314344, lj[4]:-0.17909225698562206,  lj[5]:2.0931167850696473, lj[6]:-0.34552917247118947}
    jointposR ={rj[0]:-0.41609228871391846, rj[1]:0.11313108310654926, rj[2]:1.6425099286283067, rj[3]:1.0339030510347689, rj[4]:-0.12425244381871851,  rj[5]:1.692747799431554, rj[6]:0}

    def set_j(limb, joint_name, delta):
        current_position = limb.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        limb.set_joint_positions(joint_command)

    #right.move_to_joint_positions(jointposR)
    left.move_to_joint_positions(jointposL)      
    
    cameraL = baxter_interface.CameraController('left_hand_camera')
    cameraR = baxter_interface.CameraController('right_hand_camera')

    cameraL.resolution = (960, 600,)
    cameraR.resolution = (960, 600,)

    print("Opening left_hand_camera")
    print("Opening right_hand_camera")

    cameraL.open()
    cameraR.open()
    
    RC=threading.Thread(target=RightArmControl)
    LC=threading.Thread(target=LeftArmControl)
    RC.start()
    LC.start() 
    #rospy.Subscriber('/cameras/left_hand_camera/image', Image, image_callback_left) 
    #rospy.Subscriber('/cameras/right_hand_camera/image', Image, image_callback_right) 

    def clean_shutdown():
        global Shutdown
        Shutdown =True
        print("\nExiting Project...")

        if not init_state:
            print("Disabling robot...")
            
            rs.disable()
    rospy.on_shutdown(clean_shutdown)
  

    #while(not rospy.is_shutdown()):
        
        #effort=right.endpoint_effort()
        #print(effort)
        #print("got 1")

    rospy.spin()    # sleep 
    cv2.destroyAllWindows() 

 