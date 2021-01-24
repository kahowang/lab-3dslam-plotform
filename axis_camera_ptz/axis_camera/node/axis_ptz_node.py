#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import time
import urllib
import urllib2
import threading

import rospy
from axis_ptz import axis_ptz


def main():
    rospy.init_node("axis_ptz")
    hostname = rospy.get_param("hostname", default="192.168.80.1")
    username = rospy.get_param("username", default="iqr")
    password = rospy.get_param("password", default="123")
    enable_joint = rospy.get_param("enable_joint", default=True)
    ptz_pub_rate = rospy.get_param("ptz_pub_rate", default=10)
    pan_joint = rospy.get_param("pan_joint", default="pan_joint")
    tilt_joint = rospy.get_param("tilt_joint", default="tilt_joint")

    
    def loop():
        r = rospy.Rate(ptz_pub_rate)
        p5635_ptz = axis_ptz(hostname, username, password, enable_joint, pan_joint, tilt_joint)
        while True:
            p5635_ptz.pub_ptz_msg()
            r.sleep()
    t = threading.Thread(target=loop)
    t.start()
    rospy.spin()
    

if __name__ == "__main__":
    main()