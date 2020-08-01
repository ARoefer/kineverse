#!/usr/bin/env python
import rospy

from kineverse.ros.tf_publisher import NetworkedTFBroadcaster


if __name__ == '__main__':
    rospy.init_node('kineverse_model_publisher')

    js_topic   = rospy.get_param('~state_topic', 'joint_states')
    desc_param = rospy.get_param('~robot_description', 'robot_description')
    model      = rospy.get_param('~model_path', '/')
    use_js_msg = rospy.get_param('~use_js_msg', False)

    node = NetworkedTFBroadcaster(desc_param, js_topic, model, 50, use_js_msg)

    while not rospy.is_shutdown():
        rospy.sleep(1000)
