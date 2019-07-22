import rospy
import subprocess
import os

from kineverse.gradients.gradient_math    import *
from kineverse.model.kinematic_model      import KinematicModel, Path
from kineverse.motion.integrator          import CommandIntegrator
from kineverse.motion.min_qp_builder      import TypedQPBuilder as TQPB
from kineverse.motion.min_qp_builder      import SoftConstraint, ControlledValue
from kineverse.operations.urdf_operations import load_urdf
from kineverse.type_sets                  import atomic_types
from kineverse.utils                      import res_pkg_path
from kineverse.visualization.graph_generator import generate_dependency_graph, plot_graph
from kineverse.visualization.plotting     import draw_recorders, split_recorders

from sensor_msgs.msg     import JointState as JointStateMsg

from urdf_parser_py.urdf import URDF


if __name__ == '__main__':
    rospy.init_node('kineverse_sandbox')

    plot_dir = res_pkg_path('package://kineverse/test/plots')
    pub_path = '/opt/ros/{}/lib/robot_state_publisher/robot_state_publisher'.format(os.environ['ROS_DISTRO'])

    with open(res_pkg_path('package://fetch_description/robots/fetch.urdf'), 'r') as urdf_file:
        urdf_str = urdf_file.read()

    js_pub = rospy.Publisher('/joint_states', JointStateMsg, queue_size=1)
    rospy.set_param('/robot_description', urdf_str)
    urdf_model = URDF.from_xml_string(urdf_str)

    # ROBOT STATE PUBLISHER
    sp_p = subprocess.Popen([pub_path,
                            '__name:={}_state_publisher'.format(urdf_model.name),
                            'robot_description:=/robot_description',
                            '_tf_prefix:={}'.format(urdf_model.name),
                            'joint_states:=/joint_states'])

    km = KinematicModel()
    load_urdf(km, Path('fetch'), urdf_model)
    plot_graph(generate_dependency_graph(km), '{}/sandbox_dep_graph.pdf'.format(plot_dir))


    eef_pose = km.get_data('fetch/links/gripper_link/pose')
    eef_pos  = pos_of(eef_pose)

    cam_pose    = km.get_data('fetch/links/head_camera_link/pose')
    cam_pos     = pos_of(cam_pose)
    cam_forward = x_of(cam_pose)
    cam_to_eef  = eef_pos - cam_pos

    look_goal = 1 - (dot(cam_to_eef, cam_forward) / norm(cam_to_eef))

    goal = point3(0.4, 0.4, 1.4)
    dist = norm(goal - eef_pos)

    joint_symbols = [j.position for j in km.get_data('fetch/joints').values() if hasattr(j, 'position') and type(j.position) is spw.Symbol]
    controlled_symbols = {get_diff_symbol(j) for j in joint_symbols}

    constraints = km.get_constraints_by_symbols(dist.free_symbols.union(controlled_symbols))

    controlled_values = {}
    to_remove = set()
    for k, c in constraints.items():
        if type(c.expr) is spw.Symbol and c.expr in controlled_symbols:
            controlled_values[str(c.expr)] = ControlledValue(c.lower, c.upper, c.expr, 0.01)
            to_remove.add(k)

    constraints = {k: c for k, c in constraints.items() if k not in to_remove}
    for s in controlled_symbols:
        if str(s) not in controlled_values:
            controlled_values[str(s)] = ControlledValue(-1e9, 1e9, s, 0.01)


    goal_constraints = {'reach_point': SoftConstraint(-dist, -dist, 1, dist),
                        'look_at_eef': SoftConstraint(-look_goal, -look_goal, 1, look_goal)}

    integrator = CommandIntegrator(TQPB(constraints, goal_constraints, controlled_values), recorded_terms={'distance': dist,
                                            'gaze_align': look_goal})

    int_factor = 0.02
    integrator.restart('Fetch Cartesian Goal Example')
    integrator.run(int_factor)

    draw_recorders([integrator.recorder] + split_recorders([integrator.sym_recorder]), 4.0/9.0, 8, 4).savefig('{}/fetch_sandbox_plots.png'.format(plot_dir))

    trajectory = {Path(j)[-1][:-2]: [0.0] * len(integrator.recorder.data.values()[0]) for j in joint_symbols}
    trajectory.update({Path(spw.Symbol(j))[-1][:-2]: d for j, d in integrator.recorder.data.items()})
    x = 0
    jsmsg      = JointStateMsg()
    jsmsg.name = trajectory.keys()
    while not rospy.is_shutdown() and x < len(trajectory.values()[0]):
        now = rospy.Time.now()
        if (now - jsmsg.header.stamp).to_sec() >= int_factor:
            jsmsg.header.stamp = now
            jsmsg.position = [trajectory[j][x] for j in jsmsg.name] 
            js_pub.publish(jsmsg)
            x += 1 

    sp_p.terminate()
    sp_p.wait()

