#!/usr/bin/env python

import os
import robot_upstart

j = robot_upstart.Job("AXIS_SETUP")
j.add(package="axis_camera", filename="launch/axis.launch")
j.install()
