#!/usr/bin/env python
import numpy as np

from tqdm import tqdm

from kineverse.type_sets               import symengine_types, symbolic_types
from kineverse.motion.integrator       import CommandIntegrator, DT_SYM
from kineverse.motion.min_qp_builder   import Constraint, SoftConstraint, ControlledValue
from kineverse.motion.min_qp_builder   import TypedQPBuilder as TQPB
from kineverse.gradients.diff_logic    import *
from kineverse.gradients.gradient_math import *
from kineverse.model.articulation_model   import ArticulationModel
from kineverse.visualization.plotting  import split_recorders, draw_recorders

def subs_if_sym(var, subs_dict):
    if hasattr(var, 'subs') and callable(var.subs):
        return var.subs(subs_dict)
    return var

def sign(x):
    return -1 if x < 0 else (1 if x > 0 else 0)


class Scenario(object):
    def __init__(self, name):
        self.name = name
        self.km   = ArticulationModel()
        self.qp_type           = TQPB
        self.int_rules         = {}
        self.recorded_terms    = {}
        self.hard_constraints  = {}
        self.soft_constraints  = {}
        self.controlled_values = {}
        self.start_state       = {}
        self.value_recorder    = None
        self.symbol_recorder   = None

    def run(self, integration_step=0.02, max_iterations=200):
        integrator = CommandIntegrator(self.qp_type(self.hard_constraints, self.soft_constraints, self.controlled_values), self.int_rules, self.start_state, self.recorded_terms)

        integrator.restart(self.name)
        integrator.run(integration_step, max_iterations)
        self.value_recorder  = integrator.recorder
        self.symbol_recorder = integrator.sym_recorder


class Door(Scenario):
    def __init__(self, name):
        super(Door, self).__init__(name)

        self.door_p      = create_pos('door')
        self.door_v      = get_diff_symbol(self.door_p)
        self.handle_p    = create_pos('handle')
        self.handle_v    = get_diff_symbol(self.handle_p)
        self.threshold_p = 0.5 # create_pos('gamma')
        self.locked_p    = 0.02 # create_pos('phi')

        constraint_door_p = Constraint(-0.3 - self.door_p, 1.57 - self.door_p, self.door_p)
        self.hard_constraints = {'door position': constraint_door_p}
        self.controlled_values = {  str(self.door_v): ControlledValue(-0.4, 0.4, self.door_v, 0.001),
                                  str(self.handle_v): ControlledValue(-0.4, 0.4, self.handle_v, 0.001)}

    def run(self, integration_step=0.02, max_iterations=200):
        super(Door, self).run(integration_step, max_iterations)
        self.value_recorder.add_threshold('unlock', self.threshold_p)


class NonBlockingDoor_NoAction(Door):
    def __init__(self):
        super(NonBlockingDoor_NoAction, self).__init__('Non-Blocking Door - No Action')


        condition = alg_or(greater_than(self.door_p, self.locked_p), greater_than(self.handle_p, self.threshold_p))
        self.recorded_terms = {'condition': condition.expr}
        self.start_state = {self.door_p: 0}

        self.hard_constraints['door_velocity'] = Constraint(-1, condition, self.door_p)
        self.soft_constraints = {'open_door': SoftConstraint(1.57 - self.door_p, 1.57 - self.door_p, 1, self.door_p)}


class NonBlockingDoor(NonBlockingDoor_NoAction):
    def __init__(self):
        super(NonBlockingDoor, self).__init__()
        self.name = 'Non-Blocking Door - Press Handle'

        self.soft_constraints['push_handle'] = SoftConstraint(self.threshold_p + 0.1 - self.handle_p, 1, 1, self.handle_p)


class BlockingDoor(Door):
    def __init__(self, name):
        super(BlockingDoor, self).__init__('Blocking Door - {}'.format(name))

        condition_open  = alg_or(greater_than(self.door_p, self.locked_p), greater_than(self.handle_p, self.threshold_p))
        condition_close = alg_or(less_than(self.door_p, self.locked_p), greater_than(self.handle_p, self.threshold_p))
        self.recorded_terms = {'condition_open': condition_open.expr, 
                               'condition_close': condition_close.expr}

        self.hard_constraints['door_velocity'] = Constraint(-condition_close, condition_open, self.door_p)

class BlockingDoorOpen(BlockingDoor):
    def __init__(self):
        super(BlockingDoorOpen, self).__init__('Open')

        self.soft_constraints = {
            'open_door': SoftConstraint(1.57 - self.door_p, 1.57 - self.door_p, 1, self.door_p),
            'push_handle': SoftConstraint(self.threshold_p + 0.1 - self.handle_p, 1, 1, self.handle_p)}


class BlockingDoorClose(BlockingDoor):
    def __init__(self):
        super(BlockingDoorClose, self).__init__('Close')

        self.start_state = {self.door_p: 1.2}

        self.soft_constraints = {
            'close_door': SoftConstraint(-0.3 - self.door_p, -0.3 - self.door_p, 1, self.door_p),
            'push_handle': SoftConstraint(self.threshold_p + 0.1 - self.handle_p, 1, 1, self.handle_p)}


class Lockbox(Scenario):
    def __init__(self, name):
        super(Lockbox, self).__init__('Lockbox - {}'.format(name))

        self.lock_a_p = create_pos('lock_a')
        self.lock_b_p = create_pos('lock_b')
        self.lock_c_p = create_pos('lock_c')
        self.lock_d_p = create_pos('lock_d')
        self.lock_e_p = create_pos('lock_e')
        self.lock_f_p = create_pos('lock_f')

        self.lock_a_v = get_diff_symbol(self.lock_a_p)
        self.lock_b_v = get_diff_symbol(self.lock_b_p)
        self.lock_c_v = get_diff_symbol(self.lock_c_p)
        self.lock_d_v = get_diff_symbol(self.lock_d_p)
        self.lock_e_v = get_diff_symbol(self.lock_e_p)
        self.lock_f_v = get_diff_symbol(self.lock_f_p)

        self.km = ArticulationModel()

        bc_open_threshold = 0.1
        d_open_threshold  = 0.5
        e_open_threshold  = 0.2
        f_open_threshold  = 1.2

        # Locking rules
        # b and c lock a
        # d locks b and c
        # e locks c and d
        # f locks e
        a_open_condition  = alg_and(greater_than(self.lock_b_p, bc_open_threshold), greater_than(self.lock_c_p, bc_open_threshold))
        b_open_condition  = greater_than(self.lock_d_p, d_open_threshold)
        d_open_condition  = greater_than(self.lock_e_p, e_open_threshold)
        c_open_condition  = alg_and(b_open_condition, d_open_condition)
        e_open_condition  = greater_than(self.lock_f_p, f_open_threshold)

        self.recorded_terms = {'a_condition': a_open_condition.expr,
                               'b_condition': b_open_condition.expr,
                               'c_condition': c_open_condition.expr,
                               'd_condition': d_open_condition.expr,
                               'e_condition': e_open_condition.expr}

        # Velocity constraints
        self.km.add_constraint('lock_a_velocity', 
                                Constraint(-0.4 * a_open_condition, 0.4 * a_open_condition, self.lock_a_v))
        self.km.add_constraint('lock_b_velocity', 
                                Constraint(-0.1, 0.1 * b_open_condition, self.lock_b_v))
        self.km.add_constraint('lock_c_velocity', 
                                Constraint(-0.1, 0.1 * c_open_condition, self.lock_c_v))
        self.km.add_constraint('lock_d_velocity', 
                                Constraint(-0.1, 0.1 * d_open_condition, self.lock_d_v))
        self.km.add_constraint('lock_e_velocity', 
                                Constraint(-0.1, 0.1 * e_open_condition, self.lock_e_v))
        self.km.add_constraint('lock_f_velocity', 
                                Constraint(-0.4, 0.4, self.lock_f_v))

        # Configuration space
        self.km.add_constraint('lock_b_position', Constraint(-self.lock_b_p, 0.15 - self.lock_b_p, self.lock_b_p))
        self.km.add_constraint('lock_c_position', Constraint(-self.lock_c_p, 0.15 - self.lock_c_p, self.lock_c_p))
        self.km.add_constraint('lock_d_position', Constraint(-self.lock_d_p, 0.55 - self.lock_d_p, self.lock_d_p))
        self.km.add_constraint('lock_e_position', Constraint(-self.lock_e_p, 0.25 - self.lock_e_p, self.lock_e_p))


class LockboxOpeningGenerator(Lockbox):
    def __init__(self):
        super(LockboxOpeningGenerator, self).__init__('Generated Opening')

        self.start_state = {self.lock_a_p: 0,
                            self.lock_b_p: 0,
                            self.lock_c_p: 0,
                            self.lock_d_p: 0,
                            self.lock_e_p: 0,
                            self.lock_f_p: 0}

        self.soft_constraints = lock_explorer(self.km, self.start_state, 
                                    {'open_a': SoftConstraint(1.2 - self.lock_a_p, 
                                                              1.2 - self.lock_a_p, 
                                                              1, 
                                                              self.lock_a_p)}, set())
        total_symbols = set()
        for c in self.soft_constraints.values():
            total_symbols.update(c.expr.free_symbols)
        control_symbols = {get_diff_symbol(s) for s in total_symbols}
        total_symbols.update(control_symbols)
        constraints = self.km.get_constraints_by_symbols(total_symbols)
        for n, c in constraints.items():
            if c.expr in control_symbols:
                self.controlled_values[str(c.expr)] = ControlledValue(c.lower, c.upper, c.expr, 0.001)
            else:
                self.hard_constraints[n] = c


def lock_explorer(km, state, goals, generated_constraints):

    final_goals = goals.copy()
    for n, goal in goals.items():
        symbols = goal.expr.free_symbols
        goal_sign = sign(subs_if_sym(goal.lower, state)) + sign(subs_if_sym(goal.upper, state))
        if goal_sign == 0:
            continue

        goal_expr = goal.expr
        if type(goal_expr) != GC:
            goal_expr = GC(goal_expr)
        
        goal_expr.do_full_diff()

        diff_symbols = set(goal_expr.gradients.keys())
        diff_constraints = km.get_constraints_by_symbols(diff_symbols)

        diff_value = {s: subs_if_sym(g, state) for s, g in goal_expr.gradients.items()}
        diff_sign  = {s: sign(g) * goal_sign for s, g in diff_value.items()}

        symbol_constraints = {}

        for n, c in {n: c for n, c in diff_constraints.items() if c.expr in diff_symbols}.items():
            if c.expr not in symbol_constraints:
                symbol_constraints[c.expr] = {}
            symbol_constraints[c.expr][n] = c

        blocking_constraints = {}
        for s, cd in symbol_constraints.items():
            blocking_constraints[s] = {}
            for n, c in cd.items():
                c_upper = subs_if_sym(c.upper, state)
                c_lower = subs_if_sym(c.lower, state)
                sign_u = sign(c_upper)
                sign_l = sign(c_lower)
                if diff_sign[s] > 0 and sign_u <= 0:
                    blocking_constraints[s][n] = c
                elif diff_sign[s] < 0 and sign_l >= 0:
                    blocking_constraints[s][n] = c

        new_goals = {}
        # If all symbols are blocked from going in the desired direction
        if min([len(cd) for cd in blocking_constraints.values()]) > 0:
            for s, cd in blocking_constraints.items():
                for n, c in cd.items():
                    u_const_name = 'unlock {} upper bound'.format(n)
                    l_const_name = 'unlock {} lower bound'.format(n)
                    if diff_sign[s] > 0 and type(c.upper) in symbolic_types and u_const_name not in generated_constraints:
                        new_goals[u_const_name] = SoftConstraint(less_than(c.upper, 0), 1000, goal.weight, c.upper)
                        generated_constraints.add(u_const_name)
                    elif diff_sign[s] < 0 and type(c.lower) in symbolic_types and l_const_name not in generated_constraints:
                        new_goals[l_const_name] = SoftConstraint(-1000, -greater_than(c.lower, 0), goal.weight, c.lower)
                        generated_constraints.add(l_const_name)
        
        final_goals.update(lock_explorer(km, state, new_goals, generated_constraints))

    return final_goals





if __name__ == '__main__':
    scenarios = [LockboxOpeningGenerator()] # [NonBlockingDoor_NoAction(), NonBlockingDoor(), BlockingDoorOpen(), BlockingDoorClose(), LockboxOpeningGenerator()]

    for s in tqdm(scenarios):
        s.run(0.1)

    draw_recorders(list(sum([(s.value_recorder, s.symbol_recorder) for s in scenarios], tuple())), 4.0/9.0, 8, 4).savefig('tricky_kinematics.png')
    #draw_recorders(split_recorders(list(sum([(s.value_recorder, s.symbol_recorder) for s in scenarios], tuple()))), 4.0/9.0, 8, 4).savefig('tricky_kinematics.png')
