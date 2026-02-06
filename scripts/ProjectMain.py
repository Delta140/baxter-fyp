#!/usr/bin/env python

#FYP project
#Author Alevi Bingham

import argparse

import cv2
import cv_bridge

import rospy

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION


from sensor_msgs.msg import (
    Image,
    JointState,
)

def _repub_cb(msg):
    xpub_img = rospy.Publisher(
        '/robot/xdisplay',
        Image,
        queue_size=10
        )        
    xpub_img.publish(msg)


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
        '/cameras/' + 'left_hand_camera' + "/image",
        Image,
        _repub_cb
    )

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
