#!/usr/bin/env python
import os
import rospy
import random
import subprocess
import tf

from pprint import pprint

import kineverse.json_wrapper as json

from kineverse.gradients.gradient_math             import *
from kineverse.model.paths                         import Path
from kineverse.model.frames                        import Frame
from kineverse.model.geometry_model                import GeometryModel, closest_distance
from kineverse.motion.integrator                   import CommandIntegrator, DT_SYM
from kineverse.motion.min_qp_builder               import TypedQPBuilder as TQPB, \
                                                          GeomQPBuilder  as GQPB, \
                                                          Constraint, \
                                                          generate_controlled_values, \
                                                          depth_weight_controlled_values, \
                                                          extract_expr
from kineverse.motion.min_qp_builder               import SoftConstraint, ControlledValue
from kineverse.operations.basic_operations         import CreateComplexObject
from kineverse.operations.urdf_operations          import load_urdf
from kineverse.operations.special_kinematics       import create_roomba_joint_with_symbols, \
                                                          create_omnibase_joint_with_symbols, \
                                                          RoombaJoint, \
                                                          OmnibaseJoint
from kineverse.time_wrapper                        import Time
from kineverse.type_sets                           import atomic_types
from kineverse.urdf_fix                            import urdf_filler
from kineverse.utils                               import res_pkg_path
from kineverse.visualization.graph_generator       import generate_modifications_graph,\
                                                          generate_dependency_graph,   \
                                                          plot_graph
from kineverse.visualization.bpb_visualizer        import ROSBPBVisualizer
from kineverse.visualization.plotting              import draw_recorders, split_recorders
from kineverse.visualization.trajectory_visualizer import TrajectoryVisualizer

from sensor_msgs.msg     import JointState as JointStateMsg
from trajectory_msgs.msg import JointTrajectory as JointTrajectoryMsg
from trajectory_msgs.msg import JointTrajectoryPoint as JointTrajectoryPointMsg

from urdf_parser_py.urdf import URDF

tucked_arm = {'wrist_roll_joint': 0.0, 'shoulder_pan_joint': 1.32, 'elbow_flex_joint': 1.72, 'forearm_roll_joint': 0.0, 'upperarm_roll_joint': -0.2, 'wrist_flex_joint': 1.66, 'shoulder_lift_joint': 1.4, 'torso_lift_joint': 0.2}

robot = 'pr2'

if __name__ == '__main__':
    rospy.init_node('kineverse_sandbox')

    plot_dir = res_pkg_path('package://kineverse/test/plots')

    if robot != 'pr2':
        with open(res_pkg_path('package://{r}_description/robots/{r}.urdf'.format(r=robot)), 'r') as urdf_file:
            urdf_str = urdf_file.read()
    else:
        with open(res_pkg_path('package://iai_pr2_description/robots/pr2_calibrated_with_ft2.xml'), 'r') as urdf_file:
            urdf_str = urdf_file.read()

    with open(res_pkg_path('package://iai_kitchen/urdf_obj/IAI_kitchen.urdf'), 'r') as urdf_file:
        urdf_kitchen_str = urdf_file.read() 

    urdf_model    = URDF.from_xml_string(urdf_str)
    kitchen_model = urdf_filler(URDF.from_xml_string(urdf_kitchen_str))
    
    traj_pup   = rospy.Publisher('/{}/commands/joint_trajectory'.format(urdf_model.name), JointTrajectoryMsg, queue_size=1)
    kitchen_traj_pup = rospy.Publisher('/{}/commands/joint_trajectory'.format(kitchen_model.name), JointTrajectoryMsg, queue_size=1)


    # KINEMATIC MODEL
    km = GeometryModel()
    load_urdf(km, Path(robot), urdf_model)
    load_urdf(km, Path('kitchen'), kitchen_model)

    km.clean_structure()
    km.apply_operation_before('create map', 'create {}'.format(robot), CreateComplexObject(Path('map'), Frame('')))

    if robot == 'pr2':
        base_op = create_omnibase_joint_with_symbols(Path('map/pose'), 
                                                   Path('{}/links/base_link/pose'.format(robot)),
                                                   Path('{}/joints/to_map'.format(robot)),
                                                   vector3(0,0,1),
                                                   1.0, 0.6, Path(robot))
    else:
        base_op = create_roomba_joint_with_symbols(Path('map/pose'), 
                                                   Path('{}/links/base_link/pose'.format(robot)),
                                                   Path('{}/joints/to_map'.format(robot)),
                                                   vector3(0,0,1),
                                                   vector3(1,0,0),
                                                   1.0, 0.6, Path(robot))
    km.apply_operation_after('connect map base_link', 'create {}/base_link'.format(robot), base_op)
    km.clean_structure()
    km.dispatch_events()

    visualizer = ROSBPBVisualizer('/bullet_test', base_frame='map')
    traj_vis   = TrajectoryVisualizer(visualizer)

    traj_vis.add_articulated_object(urdf_model,    km.get_data(robot))
    traj_vis.add_articulated_object(kitchen_model, km.get_data('kitchen'))

    # GOAL DEFINITION
    eef_path = Path('{}/links/gripper_link/pose'.format(robot)) if robot != 'pr2' else Path('pr2/links/r_gripper_tool_frame/pose')
    eef_pose = km.get_data(eef_path)
    eef_pos  = pos_of(eef_pose)

    cam_pose    = km.get_data('{}/links/head_camera_link/pose'.format(robot)) if robot != 'pr2' else km.get_data('pr2/links/head_mount_link/pose')
    cam_pos     = pos_of(cam_pose)
    cam_forward = x_of(cam_pose)
    cam_to_eef  = eef_pos - cam_pos


    parts = ['sink_area_trash_drawer_handle', 'iai_fridge_door_handle', 'sink_area_dish_washer_door_handle', 'sink_area_left_upper_drawer_handle', 'sink_area_left_middle_drawer_handle', 'sink_area_left_bottom_drawer_handle']
    #kitchen_path = Path('kitchen/links/sink_area_trash_drawer_handle/pose')
    #kitchen_path = Path('kitchen/links/iai_fridge_door_handle/pose')
    part = parts[1] #random.choice(parts)
    kitchen_path = Path('kitchen/links/{}/pose'.format(part))

    look_goal = 1 - (dot(cam_to_eef, cam_forward) / norm(cam_to_eef))

    goal = point3(-0.4, 0.4, 1.2)
    dist = norm(goal - eef_pos)
    obj_pose = km.get_data(kitchen_path)
    geom_distance = closest_distance(eef_pose, obj_pose, eef_path[:-1], kitchen_path[:-1])

    #exit()

    print('Symbols for subworld:\n  {}'.format('\n  '.join([str(x) for x in geom_distance.free_symbols])))
    coll_world = km.get_active_geometry(geom_distance.free_symbols)

    pprint(sorted(km._symbol_co_map.keys()))

    pprint(coll_world.names)

    # QP CONFIGURTION
    base_joint  = km.get_data('{}/joints/to_map'.format(robot))
    joint_symbols = [j.position for j in km.get_data('{}/joints'.format(robot)).values() if hasattr(j, 'position') and type(j.position) is Symbol]
    k_joint_symbols = [j.position for j in km.get_data('kitchen/joints').values() if hasattr(j, 'position') and type(j.position) is Symbol]
    
    controlled_symbols = {get_diff_symbol(j) for j in joint_symbols}.union({get_diff_symbol(j) for j in obj_pose.free_symbols})
    integration_rules = None
    if isinstance(base_joint, RoombaJoint):
        controlled_symbols |= {base_joint.lin_vel, base_joint.ang_vel}
        integration_rules = {base_joint.x_pos: base_joint.x_pos + DT_SYM * get_diff(pos_of(base_link.to_parent)[0].subs({base_joint.x_pos: 0})),
                                    base_joint.y_pos: base_joint.y_pos + DT_SYM * get_diff(pos_of(base_link.to_parent)[1].subs({base_joint.y_pos: 0})),
                                    base_joint.z_pos: base_joint.z_pos + DT_SYM * get_diff(pos_of(base_link.to_parent)[2].subs({base_joint.z_pos: 0})),
                                    base_joint.a_pos: base_joint.a_pos + DT_SYM * base_joint.ang_vel}
    elif isinstance(base_joint, OmnibaseJoint):
        controlled_symbols |= {base_joint.x_pos, base_joint.y_pos, base_joint.a_pos}


    constraints = km.get_constraints_by_symbols(geom_distance.free_symbols.union(controlled_symbols))

    controlled_values, constraints = generate_controlled_values(constraints, controlled_symbols)
    controlled_values = depth_weight_controlled_values(km, controlled_values, exp_factor=1.2)

    print('Controlled values:\n{}'.format('\n'.join([str(x) for x in controlled_values.values()])))
    print('Additional joint constraints:\n{}'.format('\n'.join([str(c) for c in constraints.values() if c.expr in controlled_symbols])))

    in_contact = less_than(geom_distance, 0.05)

    constraints.update({'controlability_{}'.format(x): Constraint(-in_contact, in_contact, s) for x, s in enumerate(obj_pose.free_symbols)})

    goal_constraints = {'reach_point': SoftConstraint(-geom_distance, -geom_distance, 1, geom_distance),
                        'look_at_eef': SoftConstraint(-look_goal, -look_goal, 1, look_goal)}
    goal_constraints.update({'open_object_{}'.format(x): SoftConstraint(0.2, 0.2, 1, s) for x, s in enumerate(obj_pose.free_symbols)})


    start_state = {s: 0.0 for s in coll_world.free_symbols}
    start_state.update({create_pos((Path(robot) + (k,)).to_symbol()): v  for k, v in tucked_arm.items()})

    base_link  = km.get_data('{}/links/base_link'.format(robot)) 
    integrator = CommandIntegrator(GQPB(coll_world, constraints, goal_constraints, controlled_values, visualizer=visualizer),
    #integrator = CommandIntegrator(TQPB(constraints, goal_constraints, controlled_values),
                                   integration_rules,
                                   start_state=start_state,
                                   recorded_terms={'distance': extract_expr(geom_distance),
                                                   'gaze_align': extract_expr(look_goal),
                                                   'in contact': in_contact,
                                                   'goal': goal_constraints.values()[0].expr,
                                                   'location_x': base_joint.x_pos,
                                                   'location_y': base_joint.y_pos,
                                                   'rotation_a': base_joint.a_pos})

    # RUN
    int_factor = 0.02
    integrator.restart('Fetch Cartesian Goal Example')
    integrator.run(int_factor, 500)

    # DRAW
    draw_recorders([integrator.recorder, integrator.sym_recorder], 4.0/9.0, 8, 4).savefig('{}/fetch_sandbox_plots.png'.format(plot_dir))

    if True:
        #traj_vis.visualize(integrator.recorder.data, hz=75)
        pass
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

    traj_vis.shutdown()    