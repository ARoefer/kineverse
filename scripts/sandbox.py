import rospy
import subprocess
import os
import tf

from kineverse.gradients.gradient_math       import *
from kineverse.model.kinematic_model         import KinematicModel, Path
from kineverse.model.frames                  import Frame
from kineverse.motion.integrator             import CommandIntegrator, DT_SYM
from kineverse.motion.min_qp_builder         import TypedQPBuilder as TQPB
from kineverse.motion.min_qp_builder         import SoftConstraint, ControlledValue
from kineverse.operations.basic_operations   import CreateComplexObject
from kineverse.operations.urdf_operations    import load_urdf
from kineverse.operations.special_kinematics import create_roomba_joint_with_symbols
from kineverse.type_sets                     import atomic_types
from kineverse.utils                         import res_pkg_path
from kineverse.visualization.graph_generator import generate_dependency_graph, plot_graph
from kineverse.visualization.plotting        import draw_recorders, split_recorders

from sensor_msgs.msg     import JointState as JointStateMsg
from trajectory_msgs.msg import JointTrajectory as JointTrajectoryMsg
from trajectory_msgs.msg import JointTrajectoryPoint as JointTrajectoryPointMsg

from urdf_parser_py.urdf import URDF


if __name__ == '__main__':
    rospy.init_node('kineverse_sandbox')

    plot_dir = res_pkg_path('package://kineverse/test/plots')
    pub_path = '/opt/ros/{}/lib/robot_state_publisher/robot_state_publisher'.format(os.environ['ROS_DISTRO'])

    with open(res_pkg_path('package://fetch_description/robots/fetch.urdf'), 'r') as urdf_file:
        urdf_str = urdf_file.read()

    rospy.set_param('/robot_description', urdf_str)
    
    js_pub     = rospy.Publisher('/joint_states', JointStateMsg, queue_size=1)
    traj_pup   = rospy.Publisher('/fetch/commands/joint_trajectory', JointTrajectoryMsg, queue_size=1)
    tf_broadcaster = tf.TransformBroadcaster()

    urdf_model = URDF.from_xml_string(urdf_str)

    # ROBOT STATE PUBLISHER
    # sp_p = subprocess.Popen([pub_path,
    #                         '__name:={}_state_publisher'.format(urdf_model.name),
    #                         'robot_description:=/robot_description',
    #                         '_tf_prefix:={}'.format(urdf_model.name),
    #                         'joint_states:=/joint_states'])

    # KINEMATIC MODEL
    km = KinematicModel()
    load_urdf(km, Path('fetch'), urdf_model)


    km.apply_operation_before(CreateComplexObject(Path('map'), Frame('')), 'create map', 'create fetch')

    roomba_op = create_roomba_joint_with_symbols(Path('map/pose'), 
                                                 Path('fetch/links/base_link/pose'),
                                                 Path('fetch/joints/to_map'),
                                                 vector3(0,0,1),
                                                 vector3(1,0,0),
                                                 1.0, 0.6, Path('fetch'))
    km.apply_operation_after(roomba_op, 'connect map base_link', 'create fetch/base_link')
    km.clean_structure()
    print('Plotting dependency graph...')
    plot_graph(generate_dependency_graph(km, {'connect': 'blue'}), '{}/sandbox_dep_graph.pdf'.format(plot_dir))
    print('Done plotting.')

    # GOAL DEFINITION
    eef_pose = km.get_data('fetch/links/gripper_link/pose')
    eef_pos  = pos_of(eef_pose)

    cam_pose    = km.get_data('fetch/links/head_camera_link/pose')
    cam_pos     = pos_of(cam_pose)
    cam_forward = x_of(cam_pose)
    cam_to_eef  = eef_pos - cam_pos

    look_goal = 1 - (dot(cam_to_eef, cam_forward) / norm(cam_to_eef))

    goal = point3(0.4, 0.4, 1.2)
    dist = norm(goal - eef_pos)

    # QP CONFIGURTION
    roomba_joint  = km.get_data('fetch/joints/to_map')
    joint_symbols = [j.position for j in km.get_data('fetch/joints').values() if hasattr(j, 'position') and type(j.position) is spw.Symbol]
    controlled_symbols = {get_diff_symbol(j) for j in joint_symbols}.union({roomba_joint.lin_vel, roomba_joint.ang_vel})

    constraints = km.get_constraints_by_symbols(dist.free_symbols.union(controlled_symbols))

    controlled_values = {}
    to_remove = set()
    for k, c in constraints.items():
        if type(c.expr) is spw.Symbol and c.expr in controlled_symbols:
            weight = 0.01 if c.expr != roomba_joint.lin_vel and c.expr != roomba_joint.ang_vel else 0.2
            controlled_values[str(c.expr)] = ControlledValue(c.lower, c.upper, c.expr, weight)
            to_remove.add(k)

    constraints = {k: c for k, c in constraints.items() if k not in to_remove}
    for s in controlled_symbols:
        if str(s) not in controlled_values:
            controlled_values[str(s)] = ControlledValue(-1e9, 1e9, s, 0.01)

    print('\n'.join(controlled_values.keys()))

    goal_constraints = {'reach_point': SoftConstraint(-dist, -dist, 1, dist),
                        'look_at_eef': SoftConstraint(-look_goal, -look_goal, 1, look_goal)}

    base_link  = km.get_data('fetch/links/base_link') 
    integrator = CommandIntegrator(TQPB(constraints, goal_constraints, controlled_values),
                                   {roomba_joint.x_pos: roomba_joint.x_pos + DT_SYM * get_diff(pos_of(base_link.to_parent)[0].subs({roomba_joint.x_pos: 0})),
                                    roomba_joint.y_pos: roomba_joint.y_pos + DT_SYM * get_diff(pos_of(base_link.to_parent)[1].subs({roomba_joint.y_pos: 0})),
                                    roomba_joint.z_pos: roomba_joint.z_pos + DT_SYM * get_diff(pos_of(base_link.to_parent)[2].subs({roomba_joint.z_pos: 0})),
                                    roomba_joint.a_pos: roomba_joint.a_pos + DT_SYM * roomba_joint.ang_vel},
                                   recorded_terms={'distance': dist.expr,
                                                   'gaze_align': look_goal.expr,
                                                   'location_x': roomba_joint.x_pos,
                                                   'location_y': roomba_joint.y_pos,
                                                   'location_z': roomba_joint.z_pos,
                                                   'rotation_a': roomba_joint.a_pos})

    # RUN
    int_factor = 0.02
    integrator.restart('Fetch Cartesian Goal Example')
    integrator.run(int_factor)

    # DRAW
    draw_recorders([integrator.recorder] + split_recorders([integrator.sym_recorder]), 4.0/9.0, 8, 4).savefig('{}/fetch_sandbox_plots.png'.format(plot_dir))

    trajectory = {Path(j)[-1][:-2]: [0.0] * len(integrator.recorder.data.values()[0]) for j in joint_symbols}
    trajectory.update({Path(spw.Symbol(j))[-1][:-2]: d for j, d in integrator.recorder.data.items()})
    base_trajectory = list(zip(integrator.sym_recorder.data['location_x'],
                               integrator.sym_recorder.data['location_y'],
                               integrator.sym_recorder.data['location_z'],
                               integrator.sym_recorder.data['rotation_a']))
    if False:
        x = 0
        jsmsg      = JointStateMsg()
        jsmsg.name = trajectory.keys()
        while not rospy.is_shutdown() and x < len(trajectory.values()[0]):
            now = rospy.Time.now()
            if (now - jsmsg.header.stamp).to_sec() >= int_factor:
                jsmsg.header.stamp = now
                jsmsg.position = [trajectory[j][x] for j in jsmsg.name] 
                js_pub.publish(jsmsg)
                # tf_broadcaster.sendTransform(base_trajectory[x][:3], 
                #                              tf.transformations.quaternion_from_euler(0,0, base_trajectory[x][3]), 
                #                              now, 'fetch/base_link', 'map')
                x += 1 
    else:
        trajmsg = JointTrajectoryMsg()
        trajmsg.header.stamp = rospy.Time.now()
        trajmsg.joint_names  = trajectory.keys()
        for x in range(len(trajectory.values()[0])):
            point = JointTrajectoryPointMsg()
            point.time_from_start = rospy.Duration(x * int_factor)
            point.positions = [trajectory[j][x] for j in trajmsg.joint_names]
            trajmsg.points.append(point)
        traj_pup.publish(trajmsg)
        while (rospy.Time.now() - trajmsg.header.stamp).to_sec() > 0.5:
            pass 

    # sp_p.terminate()
    # sp_p.wait()
