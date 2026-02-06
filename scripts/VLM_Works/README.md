The ProjectMain2_MasterVer.py is the program meant to be run on the robot, the robot runs on python 2.7 and a HTTP server needs to be created in order to
listen to the response of the VLM

The VLM is hosted on the Robotics_code.py and serves to capture a image from a USB webcamer when the robot pings the VLM, the image is then fed to the VLM and the
response returned and sent off to the robot
