#!/usr/bin/env python
from kineverse.gradients.diff_logic       import get_diff_symbol
from kineverse.gradients.gradient_math    import *
from kineverse.model.event_model          import EventModel
from kineverse.model.geometry_model       import GeometryModel, Path, Constraint
from kineverse.operations.urdf_operations import load_urdf
from kineverse.utils                      import res_pkg_path

from kineverse.motion.min_qp_builder      import TypedQPBuilder  as TQPB, \
                                                 SoftConstraint  as SC, \
                                                 ControlledValue as CV, \
                                                 generate_controlled_values
from kineverse.motion.integrator          import CommandIntegrator
from kineverse.visualization.plotting     import draw_recorders, split_recorders

from urdf_parser_py.urdf import URDF 

if __name__ == '__main__':

    micro_urdf = URDF.from_xml_file(res_pkg_path('package://kineverse/urdf/microwave.urdf'))

    km = EventModel()
    #km = GeometryModel()
    load_urdf(km, Path('microwave'), micro_urdf)

    door_position   = km.get_data('microwave/joints/door_joint').position
    button_position = km.get_data('microwave/joints/button_joint').position
    button_min      = -0.02
    door_velocity   = get_diff_symbol(door_position)
    door_accel      = get_diff_symbol(door_velocity)
    point_of_release = 0.8 * button_min
    button_pushed   = less_than(button_position, point_of_release)
    condition       = alg_and(less_than(door_position, 0.1), button_pushed)

    spring_constraint = Constraint((2 * condition) ** (-100 * door_position), 1e9, door_accel)
    km.add_constraint('spring_constraint', spring_constraint)

    vc_name, old_vel_c = km.get_constant_constraints(door_velocity).items()[0] 
    print(spring_constraint)
    lock_constraint    = Constraint(old_vel_c.lower, alg_or(greater_than(door_position, 0.1), button_pushed), old_vel_c.expr)
    km.add_constraint(vc_name, lock_constraint)

    print(lock_constraint)

    goal = 1.57 - door_position

    controlled_symbols = {get_diff_symbol(button_position), door_velocity}
    cvs, constraints = generate_controlled_values(km.get_constraints_by_symbols(controlled_symbols.union({button_position})), controlled_symbols)

    print('Retrieved Constraints:\n {}'.format('\n '.join(['{}:\n  {}'.format(k, c) for k, c in constraints.items()])))

    integrator = CommandIntegrator(TQPB(constraints, 
                                        {'goal': SC(-0.1, -0.1, 1, button_position)},
                                        cvs),
                                   start_state={button_position: 0},
                                   recorded_terms={'lock_lower': lock_constraint.lower, 
                                                   'lock_upper': lock_constraint.upper,
                                                   'spring_lower': spring_constraint.lower, })

    integrator.restart('Door spring')
    integrator.run(dt=0.05)

    integrator.recorder.add_threshold('Door release', point_of_release, 'b')

    plot_dir = res_pkg_path('package://kineverse/test/plots')
    draw_recorders([integrator.recorder] + split_recorders([integrator.sym_recorder]), 4.0/9.0, 8, 4).savefig('{}/microwave_spring.png'.format(plot_dir))
