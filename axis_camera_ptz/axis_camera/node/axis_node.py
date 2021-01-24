#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import time
import urllib
import urllib2
import threading

import rospy
from axis import axis


def main():
    rospy.init_node("axis_camera")
    hostname = rospy.get_param("hostname", default="192.168.80.1")
    username = rospy.get_param("username", default="iqr")
    password = rospy.get_param("password", default="123")
    frame_id = rospy.get_param("frame_id", default="axis_image")
    p5635 = axis(hostname, username, password, frame_id)

    while True:
        try:
            if p5635.getImage():
                p5635.pubImage()
                p5635.pubImageInfo()
        except:
            rospy.logerr("Timed out while trying to get message.")


if __name__ == "__main__":
    main()
