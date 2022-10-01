#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import division, print_function
from markinhos import Markinhos
import rospy


if __name__ == "__main__":
    rospy.init_node("robozin")
    relampago_markinhos = Markinhos()

    while not rospy.is_shutdown():
        relampago_markinhos.update()
        rospy.sleep(0.001)
