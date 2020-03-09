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
from kineverse.model.geometry_model                import GeometryModel, \
                                                          contact_geometry, \
                                                          generate_contact_model
from kineverse.motion.integrator                   import CommandIntegrator, DT_SYM
from kineverse.motion.min_qp_builder               import TypedQPBuilder as TQPB, \
                                                          GeomQPBuilder  as GQPB, \
                                                          Constraint, \
                                                          generate_controlled_values, \
                                                          depth_weight_controlled_values, \
                                                          SoftConstraint as SC, \
                                                          PID_Constraint as PIDC, \
                                                          ControlledValue, \
                                                          PANDA_LOGGING
from kineverse.operations.basic_operations         import CreateComplexObject
from kineverse.operations.urdf_operations          import load_urdf
from kineverse.operations.special_kinematics       import create_roomba_joint_with_symbols, \
                                                          create_omnibase_joint_with_symbols, \
                                                          RoombaJoint
from kineverse.time_wrapper                        import Time
from kineverse.type_sets                           import atomic_types
from kineverse.urdf_fix                            import urdf_filler
from kineverse.utils                               import res_pkg_path
from kineverse.visualization.graph_generator       import generate_modifications_graph,\
                                                          generate_dependency_graph,   \
                                                          plot_graph
from kineverse.visualization.bpb_visualizer        import ROSBPBVisualizer
from kineverse.visualization.plotting              import draw_recorders,  \
                                                          split_recorders, \
                                                          convert_qp_builder_log, \
                                                          filter_contact_symbols
from kineverse.visualization.trajectory_visualizer import TrajectoryVisualizer

from sensor_msgs.msg     import JointState as JointStateMsg
from trajectory_msgs.msg import JointTrajectory as JointTrajectoryMsg
from trajectory_msgs.msg import JointTrajectoryPoint as JointTrajectoryPointMsg

from urdf_parser_py.urdf import URDF

tucked_arm = {'wrist_roll_joint'   : 0.0,
              'shoulder_pan_joint' : 1.32,
              'elbow_flex_joint'   : 1.72,
              'forearm_roll_joint' : 0.0,
              'upperarm_roll_joint': -0.2,
              'wrist_flex_joint'   : 1.66,
              'shoulder_lift_joint': 1.4,
              'torso_lift_joint'   : 0.2}
arm_poses  = {'l_elbow_flex_joint' : -2.1213,
              'l_shoulder_lift_joint': 1.2963,
              'l_wrist_flex_joint' : -1.05,
              'r_elbow_flex_joint' : -2.1213,
              'r_shoulder_lift_joint': 1.2963,
              'r_wrist_flex_joint' : -1.05,
              'torso_lift_joint'   : 0.16825}

robot = 'pr2'
# robot = 'fetch'

use_omni =  True
# use_geom_circulation = None
# use_geom_circulation = 'linear'
# use_geom_circulation = 'cubic'
use_geom_circulation = 'cross'

def sign(x):
    return -1 if x < 0 else (1 if x > 0 else 0)

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

    urdf_model    = urdf_filler(URDF.from_xml_string(urdf_str))
    kitchen_model = urdf_filler(URDF.from_xml_string(urdf_kitchen_str))
    
    traj_pup   = rospy.Publisher('/{}/commands/joint_trajectory'.format(urdf_model.name), JointTrajectoryMsg, queue_size=1)
    kitchen_traj_pup = rospy.Publisher('/{}/commands/joint_trajectory'.format(kitchen_model.name), JointTrajectoryMsg, queue_size=1)


    # KINEMATIC MODEL
    km = GeometryModel()
    load_urdf(km, Path(robot), urdf_model)
    load_urdf(km, Path('kitchen'), kitchen_model)

    km.clean_structure()
    km.apply_operation_before('create world', 'create {}'.format(robot), CreateComplexObject(Path('world'), Frame('')))

    if robot == 'pr2' or use_omni:
        base_op = create_omnibase_joint_with_symbols(Path('world/pose'), 
                                                   Path('{}/links/{}/pose'.format(robot, urdf_model.get_root())),
                                                   Path('{}/joints/to_world'.format(robot)),
                                                   vector3(0,0,1),
                                                   1.0, 0.6, Path(robot))
    else:
        base_op = create_roomba_joint_with_symbols(Path('world/pose'), 
                                                   Path('{}/links/{}/pose'.format(robot, urdf_model.get_root())),
                                                   Path('{}/joints/to_world'.format(robot)),
                                                   vector3(0,0,1),
                                                   vector3(1,0,0),
                                                   1.0, 0.6, Path(robot))
    km.apply_operation_after('connect world {}'.format(urdf_model.get_root()), 'create {}/{}'.format(robot, urdf_model.get_root()), base_op)
    km.clean_structure()
    km.dispatch_events()

    # exit(0)
    visualizer = ROSBPBVisualizer('/bullet_test', base_frame='world')
    traj_vis   = TrajectoryVisualizer(visualizer)

    traj_vis.add_articulated_object(Path(robot),     km.get_data(robot))
    traj_vis.add_articulated_object(Path('kitchen'), km.get_data('kitchen'))

    # GOAL DEFINITION
    eef_path = Path('{}/links/gripper_link/pose'.format(robot)) if robot != 'pr2' else Path('pr2/links/r_gripper_r_finger_tip_link/pose')
    eef_pose = km.get_data(eef_path)
    eef_pos  = pos_of(eef_pose)
    print(eef_pos.free_symbols)

    cam_pose    = km.get_data('{}/links/head_camera_link/pose'.format(robot)) if robot != 'pr2' else km.get_data('pr2/links/head_mount_link/pose')
    cam_pos     = pos_of(cam_pose)
    cam_forward = x_of(cam_pose)
    cam_to_eef  = eef_pos - cam_pos
    print(cam_pos.free_symbols)

    parts = ['iai_fridge_door_handle', #]
             'fridge_area_lower_drawer_handle',#]
             'oven_area_area_left_drawer_handle',#]
             'oven_area_area_middle_lower_drawer_handle',
             'oven_area_area_middle_upper_drawer_handle',
             'oven_area_area_right_drawer_handle',
             'oven_area_oven_door_handle',
             'sink_area_dish_washer_door_handle',
             'sink_area_left_bottom_drawer_handle',
             'sink_area_left_middle_drawer_handle',
             'sink_area_left_upper_drawer_handle',
             'sink_area_trash_drawer_handle'
             ]
    #kitchen_path = Path('kitchen/links/sink_area_trash_drawer_handle/pose')
    #kitchen_path = Path('kitchen/links/iai_fridge_door_handle/pose')
    
    for part in parts:
    #part = parts[0] # random.choice(parts)
        kitchen_path = Path('kitchen/links/{}/pose'.format(part))

        obj_pose = km.get_data(kitchen_path)

        #print('Symbols for subworld:\n  {}'.format('\n  '.join([str(x) for x in geom_distance.free_symbols])))

        # QP CONFIGURTION
        base_joint    = km.get_data('{}/joints/to_world'.format(robot))
        joint_symbols = [j.position for j in km.get_data('{}/joints'.format(robot)).values() if hasattr(j, 'position') and type(j.position) is Symbol]
        k_joint_symbols = [j.position for j in km.get_data('kitchen/joints').values() if hasattr(j, 'position') and type(j.position) is Symbol]
        controlled_symbols = {get_diff_symbol(j) for j in joint_symbols}.union({get_diff_symbol(j) for j in obj_pose.free_symbols})

        base_link  = km.get_data('{}/links/{}'.format(robot, urdf_model.get_root())) 
        # print(base_link.pose.free_symbols)
        # print(base_link.to_parent.free_symbols)

        integration_rules = None
        if isinstance(base_joint, RoombaJoint):
            controlled_symbols |= {base_joint.lin_vel, base_joint.ang_vel}
            integration_rules   = {base_joint.x_pos: base_joint.x_pos + DT_SYM * get_diff(pos_of(base_link.to_parent)[0].subs({base_joint.x_pos: 0})),
                                   base_joint.y_pos: base_joint.y_pos + DT_SYM * get_diff(pos_of(base_link.to_parent)[1].subs({base_joint.y_pos: 0})),
                                   base_joint.z_pos: base_joint.z_pos + DT_SYM * get_diff(pos_of(base_link.to_parent)[2].subs({base_joint.z_pos: 0})),
                                   base_joint.a_pos: base_joint.a_pos + DT_SYM * base_joint.ang_vel}
        else:
            controlled_symbols |= {get_diff(x) for x in [base_joint.x_pos, base_joint.y_pos, base_joint.a_pos]}

        # CONTACT GEOMETRY
        robot_cp, object_cp, contact_normal = contact_geometry(eef_pose, obj_pose, eef_path[:-1], kitchen_path[:-1])
        geom_distance = dot(contact_normal, robot_cp - object_cp)
        coll_world  = km.get_active_geometry(geom_distance.free_symbols)
        
        start_state = {s: 0.0 for s in coll_world.free_symbols}
        start_state.update({s: 0.4 for s in obj_pose.free_symbols})

        # GEOMETRY NAVIGATION LOGIC
        contact_grad  = sum([sign(-start_state[s]) * vector3(*[x.diff(s) for x in object_cp[:3]]) for s in obj_pose.free_symbols], vector3(0,0,0))
        robot_grad    = sum([vector3(*[x.diff(s) * DiffSymbol(s) for x in robot_cp[:3]]) for s in robot_cp.free_symbols], vector3(0,0,0))
        ortho_vel_vec = cross(contact_grad, contact_normal) # -(contact_grad - dot(contact_grad, contact_normal) * contact_normal)
        contact_tangent = cross(ortho_vel_vec, contact_normal)
        dist_scaling    = 2 ** (-0.5*((geom_distance - 0.2) / (0.2 * 0.2))**2)
  
        if use_geom_circulation == 'linear':
          geom_distance   = norm(object_cp + contact_tangent * geom_distance - robot_cp)
        elif use_geom_circulation == 'cubic':
          geom_distance   = norm(object_cp + contact_tangent * dist_scaling - robot_cp)
        elif use_geom_circulation == 'cross':
          geom_distance   = norm(object_cp + contact_tangent * norm(ortho_vel_vec) - robot_cp)

        # PUSH CONSTRAINT GENERATION
        constraints = km.get_constraints_by_symbols(geom_distance.free_symbols.union(controlled_symbols))
        constraints.update(generate_contact_model(robot_cp, controlled_symbols, object_cp, contact_normal, obj_pose.free_symbols))
        controlled_values, constraints = generate_controlled_values(constraints, controlled_symbols)
        controlled_values = depth_weight_controlled_values(km, controlled_values, exp_factor=1.1)

        # print('Controlled values:\n{}'.format('\n'.join([str(x) for x in controlled_values.values()])))
        # print('Additional joint constraints:\n{}'.format('\n'.join([str(c) for c in constraints.values() if c.expr in controlled_symbols])))

        # CAMERA STUFF
        cam_to_obj = pos_of(obj_pose) - cam_pos
        look_goal  = 1 - (dot(cam_to_obj, cam_forward) / norm(cam_to_obj))

        # GOAL CONSTAINT GENERATION
        goal_constraints = {'reach_point': PIDC(geom_distance, geom_distance, 1, k_i=0.01),
                            'look_at_obj':   SC(   -look_goal,    -look_goal, 1, look_goal)}
        goal_constraints.update({'open_object_{}'.format(x): PIDC(s, s, 1) for x, s in enumerate(obj_pose.free_symbols)})

        in_contact = less_than(geom_distance, 0.01)
 
        def debug_draw(vis, state, cmd):
          vis.begin_draw_cycle('debug_vecs')
          s_object_cp     = subs(object_cp, state)
          s_ortho_vel_vec = subs(ortho_vel_vec, state)
          vis.draw_vector('debug_vecs', s_object_cp, subs(contact_grad, state), r=0, b=0)
          vis.draw_vector('debug_vecs', s_object_cp, subs(contact_tangent, state), r=0, b=1)
          vis.draw_vector('debug_vecs', s_object_cp, s_ortho_vel_vec, r=1, b=0)
          # print(dot(s_ortho_vel_vec, subs(contact_normal, state)))
          vis.render('debug_vecs')
        
        if robot == 'pr2':
          start_state.update({Position(Path(robot) + (k,)): v  for k, v in arm_poses.items()})
        else:
          start_state.update({Position(Path(robot) + (k,)): v  for k, v in tucked_arm.items()})

        qpb = GQPB(coll_world, constraints, goal_constraints, controlled_values, visualizer=visualizer)
        qpb._cb_draw = debug_draw
        integrator = CommandIntegrator(qpb,
        #integrator = CommandIntegrator(TQPB(constraints, goal_constraints, controlled_values),
                                       integration_rules,
                                       start_state=start_state,
                                       recorded_terms={'distance': geom_distance,
                                                       'gaze_align': look_goal,
                                                       'in contact': in_contact,
                                                       'goal': goal_constraints.values()[0].expr,
                                                       'location_x': base_joint.x_pos,
                                                       'location_y': base_joint.y_pos,
                                                       'rotation_a': base_joint.a_pos})


        # RUN
        int_factor = 0.1
        integrator.restart('{} Cartesian Goal Example'.format(robot))
        print('\n'.join(['{}: {}'.format(k, v) for k, v in integrator.integration_rules.items()]))
        #exit(0)
        try:
            integrator.run(int_factor, 500)
        except Exception as e:
            print(e)

        # DRAW
        print('Drawing recorders')
        draw_recorders([filter_contact_symbols(integrator.recorder, integrator.qp_builder), integrator.sym_recorder], 4.0/9.0, 8, 4).savefig('{}/{}_sandbox_{}_plots.png'.format(plot_dir, robot, part))
        if PANDA_LOGGING:
            rec_w, rec_b, rec_c, recs = convert_qp_builder_log(integrator.qp_builder)
            draw_recorders([rec_b, rec_c] + [r for _, r in sorted(recs.items())], 1, 8, 4).savefig('{}/{}_sandbox_{}_constraints.png'.format(plot_dir, robot, part))

        if False:
            traj_vis.visualize(integrator.recorder.data, hz=50)
            pass

        if rospy.is_shutdown():
          break

    traj_vis.shutdown()
