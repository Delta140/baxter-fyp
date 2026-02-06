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
from BaseHTTPServer import HTTPServer, BaseHTTPRequestHandler
import json
import random

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

action="Empty"

class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        global action
        # Read request body
        length = int(self.headers.getheader('Content-Length'))
        body = self.rfile.read(length)

        # Parse JSON
        try:
            data = json.loads(body)
            action=data["Action"]
            print(action)
        except:
            data = {}

        
        if(action=="Empty"):
            response = {"Ping":"True"}
        else:
            response = {"Ping":"False"}

            # Send headers
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()

            # Send body
        self.wfile.write(json.dumps(response))
        

# Run server
    
bridge = cv_bridge.CvBridge()


WindowX=440
WindowY=80
WindowSize=80

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
    
def Server():
    server = HTTPServer(("0.0.0.0", 8080), Handler)
    print("Python 2.7 server running on port 8080")
    server.serve_forever()

def LeftArmControl():
    global action
    while(Shutdown==False):
        
        Pos = left.endpoint_pose()
        CurrentPos=Pos["position"]
        CurrentOri=Pos["orientation"]
        
        PosX=CurrentPos.x
        PosY=CurrentPos.y
        PosZ=CurrentPos.z

        OriX=CurrentOri.x
        OriY=CurrentOri.y
        OriZ=CurrentOri.z
        if(action != "Empty"):
            print(action)
        if(action.upper()=="LEFT"):
            CurrentPos=CurrentPos._replace(y=PosY+0.05*(random.uniform(0.9,1.1)))
        elif(action.upper()=="RIGHT"):
            CurrentPos=CurrentPos._replace(y=PosY-0.05*(random.uniform(0.9,1.1)))
        elif(action.upper()=="UP"):
            CurrentPos=CurrentPos._replace(z=PosZ+0.05*(random.uniform(0.9,1.1)))
        elif(action.upper()=="DOWN"):
            CurrentPos=CurrentPos._replace(z=PosZ-0.05*(random.uniform(0.9,1.1)))
        elif(action.upper()=="FORWARD"):
            CurrentPos=CurrentPos._replace(x=PosX+0.05*(random.uniform(0.9,1.1)))
        elif(action.upper()=="BACKWARD"):
            CurrentPos=CurrentPos._replace(x=PosX-0.05*(random.uniform(0.9,1.1)))
        elif(action.upper()=="CLOSE_GRIP"):
            grip_left.close()

        action="Empty"

        movement=IKSolver(CurrentPos,CurrentOri, "left")
        #print(movement)
        if(movement !=0):
            left.move_to_joint_positions(movement)

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
    #jointposL ={lj[0]:-1.1800147210808545, lj[1]:-0.7850146682003605, lj[2]:0.25617479157686407, lj[3]:0.2619272195314344, lj[4]:-0.17909225698562206,  lj[5]:2.0931167850696473, lj[6]:-0.34552917247118947}

    def set_j(limb, joint_name, delta):
        current_position = limb.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        limb.set_joint_positions(joint_command)

    #right.move_to_joint_positions(jointposR)
    #left.move_to_joint_positions(jointposL)      
    

    
    SC=threading.Thread(target=Server)
    LC=threading.Thread(target=LeftArmControl)
    SC.start()
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

 