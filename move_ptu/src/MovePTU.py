#! /usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import time

def main():
    rospy.init_node('move_ptu')
    pan=rospy.get_param('pan',0)
    tilt=rospy.get_param('tilt',0)
    ptu_pub = rospy.Publisher('/ptu/cmd', JointState)
    time.sleep(10)

    msg = JointState()
    msg.name = ('head_pan_joint', 'head_tilt_joint')
    msg.position = (pan, tilt)#-0.7  #Max positive pan = 2.76ish
    msg.velocity = (1, 1)

    ptu_pub.publish(msg)

if __name__ == '__main__':
    main()
