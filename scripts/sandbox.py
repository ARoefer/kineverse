from kineverse.gradients.gradient_math    import *
from kineverse.model.kinematic_model      import KinematicModel, Path
from kineverse.motion.integrator          import CommandIntegrator
from kineverse.motion.min_qp_builder      import TypedQPBuilder as TQPB
from kineverse.motion.min_qp_builder      import SoftConstraint, ControlledValue
from kineverse.operations.urdf_operations import load_urdf
from kineverse.type_sets                  import atomic_types
from kineverse.utils                      import res_pkg_path
from kineverse.visualization.plotting     import draw_recorders

from urdf_parser_py.urdf import URDF


if __name__ == '__main__':
    km = KinematicModel()
    load_urdf(km, Path('fetch'), URDF.from_xml_file(res_pkg_path('package://fetch_description/robots/fetch.urdf')))


    eef_pose = km.get_data('fetch/links/gripper_link/pose')
    eef_pos  = pos_of(eef_pose)

    goal = point3(1, 0.3, 1)
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


    goal_constraints = {'reach_point': SoftConstraint(-dist, -dist, 1, dist)}

    integrator = CommandIntegrator(TQPB(constraints, goal_constraints, controlled_values), recorded_terms={'distance': dist})

    integrator.restart('Fetch Cartesian Goal Example')
    integrator.run(0.05)

    draw_recorders([integrator.recorder, integrator.sym_recorder], 4.0/9.0, 8, 4).savefig(res_pkg_path('package://kineverse/test/plots/fetch_sandbox_plots.png'))



