#!/usr/bin/env python
import rospy

from kineverse.network.server import ModelServer

if __name__ == '__main__':
    rospy.init_node('kineverse_server')

    server = ModelServer()

    while not rospy.is_shutdown():
        rospy.sleep(1000)