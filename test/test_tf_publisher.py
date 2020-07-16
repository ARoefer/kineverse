#!/usr/bin/env python
import rospy

from kineverse.ros.tf_publisher import NetworkedTFBroadcaster
from kineverse_msgs.msg         import ValueMap   as ValueMapMsg

from math import sin
from time import time

if __name__ == '__main__':
    rospy.init_node('kineverse_model_publisher')

    js_topic   = 'joint_states'
    desc_param = 'iai_oven_area/robot_description'
    model      = 'iai_oven_area'
    use_js_msg = False

    node = NetworkedTFBroadcaster(desc_param, js_topic, model, 50, use_js_msg)

    msg = ValueMapMsg()
    msg.symbol = None
    msg.value  = None

    while not rospy.is_shutdown():
        if node.model is not None and msg.symbol is None:
            msg.symbol = [str(s) for s in node.s_frame_map.keys()]
            print(msg.symbol)

        if msg.symbol is not None:
            msg.value = [0.4 + 0.4 * sin(time() + 0.2 * x) for x in range(len(msg.symbol))]
            node.cb_state_update(msg)
