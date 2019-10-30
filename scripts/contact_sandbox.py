#!/usr/bin/env python
import rospy

from kineverse.gradients.gradient_math      import *
from kineverse.model.geometry_model         import GeometryModel, \
                                                   RigidBody, \
                                                   Geometry, \
                                                   Constraint, \
                                                   contact_geometry, \
                                                   generate_contact_model, \
                                                   Path
from kineverse.motion.min_qp_builder        import SoftConstraint as SC,   \
                                                   TypedQPBuilder as TQPB, \
                                                   PIDQPBuilder   as PQPB, \
                                                   GeomQPBuilder  as GQPB, \
                                                   PID_Constraint as PIDC, \
                                                   generate_controlled_values, \
                                                   default_bound
from kineverse.motion.integrator            import CommandIntegrator
from kineverse.network.ros_conversion       import auto_encode, PointMsg, Vector3Msg, PoseMsg
from kineverse.visualization.bpb_visualizer import ROSBPBVisualizer
from kineverse.visualization.plotting       import draw_recorders, convert_qp_builder_log
from kineverse.utils                        import res_pkg_path

from math import pi

zero_point = point3(0,0,0)

if __name__ == '__main__':
    rospy.init_node('contact_sandbox')

    km = GeometryModel()
    visualizer = ROSBPBVisualizer('/debug_vis', 'map')

    eef_start  = -1
    cube_goal  = point3(0.5, 0.5, 0)
    cube_width = 0.1
    eef_width  = 0.1
    cube_geom  = Geometry('cube', spw.eye(4), 'box', vector3(cube_width, 1, 1))
    eef_geom   = Geometry('eef', spw.eye(4), 'sphere', vector3(*[eef_width]*3))

    cube_pos_x = Position('cube_x')
    cube_pos_y = Position('cube_y')
    cube_pos_a = Position('cube_a')
    # cube_frame = translation3(cube_pos_x, cube_pos_y, 0) # rotation3_axis_angle(vector3(0,0,1), cube_pos) 
    # cube_frame = rotation3_axis_angle(vector3(0,0,1), cube_pos_y) 
    cube_frame = translation3(0, 0, 0) * rotation3_axis_angle(vector3(0,0,1), cube_pos_a)

    cube_rb    = RigidBody('map', cube_frame, {0: cube_geom}, {0: cube_geom})
    cube_path  = Path('cube')
    cube_err   = -pi * 0.5 - cube_pos_a

    eef_pos    = Position('eef')
    eef_pos_y  = Position('eef_y')
    eef_frame  = rotation3_rpy(0, 0, pi * 0) * translation3(eef_pos, eef_pos_y, 0.2)
    eef_rb     = RigidBody('map', eef_frame, {0: eef_geom}, {0: eef_geom})
    eef_path   = Path('eef')

    km.set_data(cube_path, cube_rb)
    km.set_data(eef_path,  eef_rb)
    km.clean_structure()
    km.dispatch_events()

    eef_point, contact, contact_normal = contact_geometry(eef_frame, cube_frame, eef_path, cube_path)

    constraints = generate_contact_model(eef_point, {get_diff(x) for x in eef_frame.free_symbols}, contact, contact_normal, cube_frame.free_symbols, 0.02, set_inanimate=True)
    constraints.update({'vel_cube': Constraint(-0.2, 0.2, get_diff(norm(pos_of(cube_frame)))),
                        'vel_eef_y':  Constraint(-0.4, 0.4, eef_pos_y),
                        'vel_eef_x':  Constraint(-0.4, 0.4, eef_pos)})

    cvs, constraints = generate_controlled_values(constraints, {get_diff(x) for x in eef_frame.free_symbols.union(cube_frame.free_symbols)})

    surface_dist     = dot(contact_normal, eef_point - contact)
    soft_constraints = {'make_contact': PIDC(surface_dist, surface_dist, 1, k_i=0.1),
                        'move_cube':    SC(-cube_err, -cube_err, 1, cube_err)
                        }# k_i=0)}

    sub_world = km.get_active_geometry(eef_frame.free_symbols.union(cube_frame.free_symbols))

    recorded_terms = {'contact distance': surface_dist,
                      'cube_pos_x': pos_of(cube_frame)[0],
                      'cube_pos_x': pos_of(cube_frame)[1],
                      'cube_err': cube_err
                       }
    recorded_terms.update({k: c.expr for k, c in constraints.items() if k[:17] == 'motion_alignment_'})
    int_rules  = {get_diff(cube_pos_x): get_diff(cube_pos_x),
                  get_diff(cube_pos_y): get_diff(cube_pos_y)}
    integrator = CommandIntegrator(GQPB(sub_world, constraints, soft_constraints, cvs, visualizer=visualizer),
                                   start_state={eef_pos: eef_start,
                                                eef_pos_y: 0.2},
                                   recorded_terms=recorded_terms,
                                   integration_rules=int_rules,
                                   equilibrium=0.01)
    integrator.restart('Push model')
    try:
        integrator.run(0.05)
    except Exception as e:
        print(e)

    plot_path = res_pkg_path('package://kineverse/test/plots')
    draw_recorders([integrator.recorder, integrator.sym_recorder], 0.5, 8, 4).savefig('{}/push_model.png'.format(plot_path))
    rec_w, rec_b, rec_c, recs = convert_qp_builder_log(integrator.qp_builder)
    draw_recorders([rec_b, rec_c] + [r for _, r in sorted(recs.items())], 1, 8, 4).savefig('{}/push_constraints.png'.format(plot_path))
