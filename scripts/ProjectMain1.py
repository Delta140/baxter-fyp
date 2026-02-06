#!/usr/bin/env python

#FYP project
#Author Alevi Bingham

import argparse

import cv2
import cv_bridge

import rospy

import baxter_interface
import baxter_external_devices

import math 

from baxter_interface import CHECK_VERSION



from sensor_msgs.msg import (
    Image,
    JointState,
)

bridge = cv_bridge.CvBridge()

face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier('haarcascade_eye.xml')

def _repub_cb(msg):
    xpub_img = rospy.Publisher(
        '/robot/xdisplay',
        Image,
        queue_size=10
        )        
    xpub_img.publish(msg)

def image_callback(ros_img):
    cv_image = bridge.imgmsg_to_cv2(ros_img, desired_encoding="passthrough")
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    for (x,y,w,h) in faces:
        cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = cv_image[y:y+h, x:x+w]
        eyes = eye_cascade.detectMultiScale(roi_gray)
        for (ex,ey,ew,eh) in eyes:
            cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
    NewRos_image = bridge.imgmsg_to_cv2(cv_image, desired_encoding="passthrough")
    xpub_img = rospy.Publisher(
        '/robot/xdisplay',
        Image,
        queue_size=10
        )
    xpub_img.publish(NewRos_image)    


def main():
    
    epilog = """
    test
    """
    
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("Project1")
    print("Getting robot state... ")

    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    camera1 = baxter_interface.CameraController('left_hand_camera')
    camera1.resolution = (960, 600,)
    print("Opening left_hand_camera")
    camera1.open()
    print("Display left_hand_camera")
    rospy.Subscriber(
        '/cameras/' + 'left_hand_camera' + '/image',
        Image,
        image_callback
    )
    cv2.destroyAllWindows() 
    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

if __name__ == '__main__':
    main()
