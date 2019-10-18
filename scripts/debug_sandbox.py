#!/usr/bin/env python
import rospy
import subprocess
import os
import tf
import random

import kineverse.json_wrapper as json

from kineverse.gradients.gradient_math       import *
from kineverse.model.paths                   import Path
from kineverse.model.frames                  import Frame
from kineverse.model.geometry_model          import GeometryModel, closest_distance
from kineverse.motion.integrator             import CommandIntegrator, DT_SYM
from kineverse.motion.min_qp_builder         import TypedQPBuilder as TQPB, \
                                                    GeomQPBuilder  as GQPB
from kineverse.motion.min_qp_builder         import SoftConstraint, ControlledValue
from kineverse.operations.basic_operations   import CreateComplexObject
from kineverse.operations.urdf_operations    import load_urdf
from kineverse.operations.special_kinematics import create_roomba_joint_with_symbols
from kineverse.time_wrapper                  import Time
from kineverse.type_sets                     import atomic_types
from kineverse.urdf_fix                      import urdf_filler
from kineverse.utils                         import res_pkg_path
from kineverse.visualization.graph_generator import generate_modifications_graph,\
                                                    generate_dependency_graph,   \
                                                    plot_graph
from kineverse.visualization.plotting        import draw_recorders, split_recorders
from kineverse.visualization.bpb_visualizer  import ROSBPBVisualizer

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

    with open(res_pkg_path('package://iai_kitchen/urdf_obj/IAI_kitchen.urdf'), 'r') as urdf_file:
        urdf_kitchen_str = urdf_file.read() 

    urdf_model    = URDF.from_xml_string(urdf_str)
    kitchen_model = urdf_filler(URDF.from_xml_string(urdf_kitchen_str))
    
    js_pub     = rospy.Publisher('/{}/joint_states'.format(urdf_model.name), JointStateMsg, queue_size=1)
    traj_pup   = rospy.Publisher('/{}/commands/joint_trajectory'.format(urdf_model.name), JointTrajectoryMsg, queue_size=1)
    kitchen_js_pub   = rospy.Publisher('/{}/joint_states'.format(kitchen_model.name), JointStateMsg, queue_size=1)
    kitchen_traj_pup = rospy.Publisher('/{}/commands/joint_trajectory'.format(kitchen_model.name), JointTrajectoryMsg, queue_size=1)
    tf_broadcaster = tf.TransformBroadcaster()


    rospy.set_param('/{}/robot_description'.format(urdf_model.name), urdf_str)
    rospy.set_param('/{}/robot_description'.format(kitchen_model.name), urdf_kitchen_str)
    # ROBOT STATE PUBLISHER
    sp_fp = subprocess.Popen([pub_path,
                            '__name:={}_state_publisher'.format(urdf_model.name),
                            'robot_description:=/{}/robot_description'.format(urdf_model.name),
                            '_tf_prefix:={}'.format(urdf_model.name),
                            'joint_states:=/{}/joint_states'.format(urdf_model.name)])
    sp_kp = subprocess.Popen([pub_path,
                            '__name:={}_state_publisher'.format(kitchen_model.name),
                            'robot_description:=/{}/robot_description'.format(kitchen_model.name),
                            '_tf_prefix:={}'.format(kitchen_model.name),
                            'joint_states:=/{}/joint_states'.format(kitchen_model.name)])

    # KINEMATIC MODEL
    km = GeometryModel()
    load_urdf(km, Path('fetch'), urdf_model)
    load_urdf(km, Path('kitchen'), kitchen_model)

    km.clean_structure()
    km.apply_operation_before('create map', 'create fetch', CreateComplexObject(Path('map'), Frame('')))

    roomba_op = create_roomba_joint_with_symbols(Path('map/pose'), 
                                                 Path('fetch/links/base_link/pose'),
                                                 Path('fetch/joints/to_map'),
                                                 vector3(0,0,1),
                                                 vector3(1,0,0),
                                                 1.0, 0.6, Path('fetch'))
    km.apply_operation_after('connect map base_link', 'create fetch/base_link', roomba_op)
    km.clean_structure()
    km.dispatch_events()

    print('Plotting dependency graph...')
    #plot_graph(generate_dependency_graph(km, {'connect': 'blue'}), '{}/sandbox_dep_graph.pdf'.format(plot_dir))
    #plot_graph(generate_modifications_graph(km, {'connect': 'blue'}), '{}/sandbox_mod_graph.pdf'.format(plot_dir))
    print('Done plotting.')

    # GOAL DEFINITION
    eef_path = Path('fetch/links/gripper_link/pose')
    eef_pose = km.get_data(eef_path)
    eef_pos  = pos_of(eef_pose)

    cam_pose    = km.get_data('fetch/links/head_camera_link/pose')
    cam_pos     = pos_of(cam_pose)
    cam_forward = x_of(cam_pose)
    cam_to_eef  = eef_pos - cam_pos


    parts = ['sink_area_trash_drawer_handle', 'iai_fridge_door_handle', 'sink_area_dish_washer_door_handle', 'sink_area_left_upper_drawer_handle', 'sink_area_left_middle_drawer_handle', 'sink_area_left_bottom_drawer_handle']
    #kitchen_path = Path('kitchen/links/sink_area_trash_drawer_handle/pose')
    #kitchen_path = Path('kitchen/links/iai_fridge_door_handle/pose')
    part = random.choice(parts)
    kitchen_path = Path('kitchen/links/{}/pose'.format(part))

    look_goal = 1 - (dot(cam_to_eef, cam_forward) / norm(cam_to_eef))

    goal = point3(-0.4, 0.4, 1.2)
    dist = norm(goal - eef_pos)
    obj_pose = km.get_data(kitchen_path)
    geom_distance = closest_distance(eef_pose, obj_pose, eef_path[:-1], kitchen_path[:-1])


    print('Symbols for subworld:\n  {}'.format('\n  '.join([str(x) for x in geom_distance.free_symbols])))
    coll_world = km.get_active_geometry(geom_distance.free_symbols)

    # QP CONFIGURTION
    roomba_joint  = km.get_data('fetch/joints/to_map')
    joint_symbols = [j.position for j in km.get_data('fetch/joints').values() if hasattr(j, 'position') and type(j.position) is spw.Symbol]
    k_joint_symbols = [j.position for j in km.get_data('kitchen/joints').values() if hasattr(j, 'position') and type(j.position) is spw.Symbol]
    controlled_symbols = {get_diff_symbol(j) for j in joint_symbols}.union({roomba_joint.lin_vel, roomba_joint.ang_vel}).union({get_diff_symbol(j) for j in obj_pose.free_symbols})

    constraints = km.get_constraints_by_symbols(geom_distance.free_symbols.union(controlled_symbols))

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


    in_contact = less_than(geom_distance, 0.05)

    goal_constraints = {'reach_point': SoftConstraint(-geom_distance, -geom_distance, 1, geom_distance),
                        'look_at_eef': SoftConstraint(-look_goal, -look_goal, 1, look_goal)}
    goal_constraints.update({'open_object_{}'.format(x): SoftConstraint(0.2 * in_contact, 0.2 * in_contact, 1, s) for x, s in enumerate(obj_pose.free_symbols)})

    visualizer = ROSBPBVisualizer('/bullet_test', base_frame='map')

    base_link  = km.get_data('fetch/links/base_link') 
    integrator = CommandIntegrator(GQPB(coll_world, constraints, goal_constraints, controlled_values, visualizer=visualizer),
                                   {roomba_joint.x_pos: roomba_joint.x_pos + DT_SYM * get_diff(pos_of(base_link.to_parent)[0].subs({roomba_joint.x_pos: 0})),
                                    roomba_joint.y_pos: roomba_joint.y_pos + DT_SYM * get_diff(pos_of(base_link.to_parent)[1].subs({roomba_joint.y_pos: 0})),
                                    roomba_joint.z_pos: roomba_joint.z_pos + DT_SYM * get_diff(pos_of(base_link.to_parent)[2].subs({roomba_joint.z_pos: 0})),
                                    roomba_joint.a_pos: roomba_joint.a_pos + DT_SYM * roomba_joint.ang_vel},
                                   start_state={s: 0.0 for s in coll_world.free_symbols},
                                   recorded_terms={'distance': dist.expr,
                                                   'gaze_align': look_goal.expr,
                                                   'location_x': roomba_joint.x_pos,
                                                   'location_y': roomba_joint.y_pos,
                                                   'location_z': roomba_joint.z_pos,
                                                   'rotation_a': roomba_joint.a_pos})

    # RUN
    int_factor = 0.02
    integrator.restart('Fetch Cartesian Goal Example')
    integrator.run(int_factor, 500)

    # DRAW
    draw_recorders([integrator.recorder] + split_recorders([integrator.sym_recorder]), 4.0/9.0, 8, 4).savefig('{}/fetch_sandbox_plots.png'.format(plot_dir))

    f_trajectory = {Path(j)[-1][:-2]: [0.0] * len(integrator.recorder.data.values()[0]) for j in joint_symbols}
    f_trajectory.update({Path(spw.Symbol(j))[-1][:-2]: d for j, d in integrator.recorder.data.items() if Path(spw.Symbol(j))[-1][:-2] in f_trajectory})
    k_trajectory = {Path(j)[-1][:-2]: [0.0] * len(integrator.recorder.data.values()[0]) for j in k_joint_symbols}
    k_trajectory.update({Path(spw.Symbol(j))[-1][:-2]: d for j, d in integrator.recorder.data.items() if Path(spw.Symbol(j))[-1][:-2] in k_trajectory})
    base_trajectory = list(zip(integrator.sym_recorder.data['location_x'],
                               integrator.sym_recorder.data['location_y'],
                               integrator.sym_recorder.data['location_z'],
                               integrator.sym_recorder.data['rotation_a']))
    if True:
        x = 0
        f_jsmsg      = JointStateMsg()
        f_jsmsg.name = f_trajectory.keys()
        k_jsmsg      = JointStateMsg()
        k_jsmsg.name = k_trajectory.keys()
        while not rospy.is_shutdown() and x < len(f_trajectory.values()[0]):
            now = rospy.Time.now()
            if (now - f_jsmsg.header.stamp).to_sec() >= int_factor:
                f_jsmsg.header.stamp = now
                f_jsmsg.position = [f_trajectory[j][x] for j in f_jsmsg.name] 
                js_pub.publish(f_jsmsg)
                k_jsmsg.header.stamp = now
                k_jsmsg.position = [k_trajectory[j][x] for j in k_jsmsg.name] 
                kitchen_js_pub.publish(k_jsmsg)
                tf_broadcaster.sendTransform(base_trajectory[x][:3], 
                                              tf.transformations.quaternion_from_euler(0,0, base_trajectory[x][3]), 
                                              now, '{}/{}'.format(urdf_model.name, urdf_model.get_root()), 'map')
                tf_broadcaster.sendTransform((0,0,0), 
                                              tf.transformations.quaternion_from_euler(0,0,0), 
                                              now, '{}/{}'.format(kitchen_model.name, kitchen_model.get_root()), 'map')
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

    sp_fp.terminate()
    sp_fp.wait()
    sp_kp.terminate()
    sp_kp.wait()
