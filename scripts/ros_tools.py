#!/usr/bin/env python
import rospy
import sys

import kineverse.network.names as stn

from kineverse.srv import ListPaths as ListPathsSrv

if __name__ == '__main__':
    rospy.init_node('kineverse_ros_tools')

    root = sys.argv[1] if len(sys.argv) > 1 else ''
    depth = int(sys.argv[2]) if len(sys.argv) > 2 else 0

    listpaths = rospy.ServiceProxy(stn.srv_list_paths, ListPathsSrv)
    print('\n'.join(listpaths(root, depth).paths))