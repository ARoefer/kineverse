import rospy
import os
import tf
import subprocess

from kineverse.model.kinematic_model      import Path
from kineverse.model.geometry_model       import GeometryModel   
from kineverse.operations.urdf_operations import load_urdf
from kineverse.utils                      import res_pkg_path
from kineverse.visualization.graph_generator import generate_dependency_graph, plot_graph
from kineverse.visualization.plotting        import draw_recorders, split_recorders
from kineverse.urdf_fix import urdf_filler

from sensor_msgs.msg     import JointState as JointStateMsg
from urdf_parser_py.urdf import URDF



if __name__ == '__main__':
    rospy.init_node('kineverse_kitchen_sandbox')

    plot_dir = res_pkg_path('package://kineverse/test/plots')
    pub_path = '/opt/ros/{}/lib/robot_state_publisher/robot_state_publisher'.format(os.environ['ROS_DISTRO'])

    with open(res_pkg_path('package://faculty_lounge_kitchen_description/urdfs/kitchen.urdf'), 'r') as urdf_file:
        kitchen_urdf_str = urdf_file.read()

    with open(res_pkg_path('package://fetch_description/robots/fetch.urdf'), 'r') as urdf_file:
        fetch_urdf_str = urdf_file.read()

    kitchen_urdf_model = urdf_filler(URDF.from_xml_string(kitchen_urdf_str))
    rospy.set_param('/{}/robot_description'.format(kitchen_urdf_model.name), kitchen_urdf_str)
    
    kitchen_js_pub = rospy.Publisher('/{}/joint_states'.format(kitchen_urdf_model.name), JointStateMsg, queue_size=1)
    tf_broadcaster = tf.TransformBroadcaster()

    # KINEMATIC MODEL
    km = GeometryModel()
    kitchen_prefix = Path(kitchen_urdf_model.name)
    load_urdf(km, kitchen_prefix, kitchen_urdf_model)

    plot_graph(generate_dependency_graph(km, {'connect': 'blue'}), '{}/kitchen_sandbox_dep_graph.pdf'.format(plot_dir))

    # ROBOT STATE PUBLISHER
    sp_p = subprocess.Popen([pub_path,
                            '__name:={}_state_publisher'.format(kitchen_urdf_model.name),
                            'robot_description:=/{}/robot_description'.format(kitchen_urdf_model.name),
                            '_tf_prefix:={}'.format(kitchen_urdf_model.name),
                            'joint_states:=/{}/joint_states'.format(kitchen_urdf_model.name)])

    jsmsg = JointStateMsg()
    jsmsg.name = kitchen_urdf_model.joint_map.keys()
    jsmsg.position = [0.0] * len(jsmsg.name)

    int_factor = 0.02
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        if (now - jsmsg.header.stamp).to_sec() >= int_factor:
            jsmsg.header.stamp = now
            jsmsg.position = [trajectory[j][x] for j in jsmsg.name] 
            kitchen_js_pub.publish(jsmsg)

    sp_p.terminate()
    sp_p.wait()