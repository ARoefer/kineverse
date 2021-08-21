import numpy  as np
import pandas as pd

import kineverse.gradients.gradient_math  as cm

from kineverse.motion.qp_solver             import QPSolver, QPSolverException

from kineverse.gradients.diff_logic         import get_symbol_type, IntSymbol, Symbol
from kineverse.gradients.gradient_container import GradientContainer as GC
from kineverse.gradients.gradient_math      import extract_expr, wrap_expr
from kineverse.model.articulation_model     import Constraint, Path
from kineverse.model.geometry_model         import CollisionSubworld, ContactHandler, obj_to_obj_infix
from kineverse.visualization.bpb_visualizer import ROSBPBVisualizer
from kineverse.bpb_wrapper                  import transform_to_matrix as tf2mx
from kineverse.type_sets                    import is_symbolic

default_bound  = 1e9    

HardConstraint = Constraint

PANDA_LOGGING  = False

class SoftConstraint(Constraint):
    def __init__(self, lower, upper, weight, expr):
        super(SoftConstraint, self).__init__(lower, upper, expr)
        self.weight_id = weight

    @classmethod
    def from_constraint(cls, constraint, weight=1):
        return cls(constraint.lower, constraint.upper, weight, constraint.expr)

    def __str__(self):
        return '{} @ {}'.format(super(SoftConstraint, self).__str__(), self.weight_id)

    def __eq__(self, other):
        if isinstance(other, SoftConstraint):
            return c.eq_expr(self.lower, other.lower) and \
                   c.eq_expr(self.upper, other.upper) and \
                   c.eq_expr(self.expr, other.expr)   and \
                   c.eq_expr(self.weight_id, other.weight_id)
        return False


class ControlledValue(object):
    def __init__(self, lower, upper, symbol, weight=1):
        self.lower  = lower
        self.upper  = upper
        self.symbol = symbol
        self.weight_id = weight

    def __str__(self):
        return '{} <= {} <= {} @ {}'.format(self.lower, self.symbol, self.upper, self.weight_id)

    def __eq__(self, other):
        if isinstance(other, ControlledValue):
            return cm.eq_expr(self.lower, other.lower)   and \
                   cm.eq_expr(self.upper, other.upper)   and \
                   cm.eq_expr(self.symbol, other.symbol) and \
                   cm.eq_expr(self.weight, other.weight)
        return False


class MinimalQPBuilder(object):
    def __init__(self, hard_constraints, soft_constraints, controlled_values):
        hc = [(k, Constraint(extract_expr(c.lower), extract_expr(c.upper), wrap_expr(c.expr))) for k, c in hard_constraints.items()]
        sc = [(k, SoftConstraint(extract_expr(c.lower), extract_expr(c.upper), c.weight_id, wrap_expr(c.expr))) for k, c in soft_constraints.items()]
        cv = [(k, ControlledValue(extract_expr(c.lower), extract_expr(c.upper), c.symbol, extract_expr(c.weight_id))) for k, c in controlled_values.items()]

        self.np_g = np.zeros(len(cv + sc))
        self.H    = cm.diag(*[extract_expr(c.weight_id) for _, c in cv + sc])
        self.lb   = cm.Matrix([c.lower if c.lower is not None else -default_bound for _, c in cv] + [-default_bound] * len(sc))
        self.ub   = cm.Matrix([c.upper if c.upper is not None else  default_bound for _, c in cv] + [default_bound] * len(sc))
        self.lbA  = cm.Matrix([c.lower if c.lower is not None else -default_bound for _, c in hc + sc])
        self.ubA  = cm.Matrix([c.upper if c.upper is not None else  default_bound for _, c in hc + sc])

        M_cv      = [c.symbol for _, c in cv]
        self.A    = cm.hstack(cm.Matrix([[c.expr[s] if s in c.expr else 0 for s in M_cv] for _, c in hc + sc]), 
                              cm.vstack(cm.zeros(len(hc), len(sc)), cm.eye(len(sc))))

        self.cv        = [c.symbol for _, c in cv]
        self.n_cv      = len(cv)
        self.n_hc      = len(hc)
        self.n_sc      = len(sc)
        self.row_names = [k for k, _ in hc + sc]
        self.col_names = [k for k, _ in cv + sc]
        self.A_dfs     = []
        self.H_dfs     = []
        self._cmd_log  = []
        self.latest_error = None

        self._build_M()

    def _build_M(self):
        self.big_ass_M = cm.vstack(cm.hstack(self.A, self.lbA, self.ubA), 
                                   cm.hstack(self.H, self.lb,  self.ub))

        self.free_symbols     = cm.free_symbols(self.big_ass_M)
        self.cython_big_ass_M = cm.speed_up(self.big_ass_M, self.free_symbols)

        self.shape1    = self.A.shape[0]
        self.shape2    = self.A.shape[1]

        self.reset_solver()

    def reset_solver(self):
        self.qp_solver = QPSolver(self.A.shape[1], self.A.shape[0])
        self.latest_error = None

    @profile
    def get_cmd(self, substitutions, nWSR=None, deltaT=None):
        substitutions = {str(s): v for s, v in substitutions.items()}
        np_big_ass_M = np.nan_to_num(self.cython_big_ass_M(**substitutions), copy=False)
        self.np_H   = np_big_ass_M[self.shape1:, :-2].copy()
        self.np_A   = np_big_ass_M[:self.shape1, :self.shape2].copy()
        self.np_lb  = np_big_ass_M[self.shape1:, -2].copy()
        self.np_ub  = np_big_ass_M[self.shape1:, -1].copy()
        self.np_lbA = np_big_ass_M[:self.shape1, -2].copy()
        self.np_ubA = np_big_ass_M[:self.shape1, -1].copy()

        self._post_process_matrices(deltaT)

        if PANDA_LOGGING:
            dfH = pd.DataFrame(np.vstack((self.np_lb, self.np_ub, self.np_H.diagonal())), index=['lb', 'ub', 'weight'], columns=self.col_names)
            dfA = pd.DataFrame(np.hstack((self.np_lbA.reshape((self.shape1, 1)), 
                                          self.np_ubA.reshape((self.shape1, 1)), 
                                          self.np_A)),
                                          index=self.row_names, columns=['lbA', 'ubA'] + self.col_names) # [:self.n_cv]

        try:
            xdot_full = self.qp_solver.solve(self.np_H, self.np_g, self.np_A, self.np_lb,  self.np_ub, 
                                                                    self.np_lbA, self.np_ubA, nWSR)
            self.latest_error = np.sum(xdot_full[len(self.cv):])
            if PANDA_LOGGING:
                self.A_dfs.append(dfA)
                self.H_dfs.append(dfH)
            self._cmd_log.append(xdot_full[:self.n_cv])
        except QPSolverException as e:
            if not PANDA_LOGGING:
                dfH = pd.DataFrame(np.vstack((self.np_lb, self.np_ub, self.np_H.diagonal())), index=['lb', 'ub', 'weight'], columns=self.col_names)
                dfA = pd.DataFrame(np.hstack((self.np_lbA.reshape((self.shape1, 1)), 
                                          self.np_ubA.reshape((self.shape1, 1)), 
                                          self.np_A[:, :-self.n_sc])), 
                                          index=self.row_names, columns=['lbA', 'ubA'] + self.col_names[:self.n_cv])

            print('INFEASIBLE CONFIGURATION!\nH:{}\nA:\n{}\nFull data written to "solver_crash_H.csv" and "solver_crash_A.csv"'.format(dfH.T, dfA))
            dfH.to_csv('solver_crash_H.csv')
            dfA.to_csv('solver_crash_A.csv')
            b_comp  = np.greater(self.np_lb,  self.np_ub)
            bA_comp = np.greater(self.np_lbA, self.np_ubA)
            if b_comp.max() or bA_comp.max():
                print('Overlapping boundary conditions:\n{}\n{}'.format('\n'.join(['{}:\n  lb: {}\n  ub: {}'.format(n, lb, ub) for n, lb, ub, c in zip(self.col_names, self.np_lb, self.np_ub, b_comp) if c]),
                     '\n'.join(['{}:\n  lbA: {}\n  ubA: {}'.format(n, lbA, ubA) for n, lbA, ubA, c in zip(self.row_names, self.np_lbA, self.np_ubA, bA_comp) if c])))
            else:
                unsat_rows = [(n, lb, ub) for n, lb, ub, d in zip(self.row_names, self.np_lbA, self.np_ubA, self.np_A) if (lb > 0 or ub < 0) and (d == 0).min()]
                if len(unsat_rows) > 0:
                    print('Direct insatisfiability:\n{}'.format('\n'.join(['{} needs change but the Jacobian is 0. lbA: {}, ubA: {}'.format(n, lb, ub) for n, lb, ub in unsat_rows])))
                else:
                    print('Boundaries are not overlapping and direct insatisfiability could not be found. Error must be more complex.')

            raise e
        if xdot_full is None:
            return None

        return {cv: xdot_full[i] for i, cv in enumerate(self.cv)}

    def equilibrium_reached(self, low_eq=1e-3, up_eq=-1e-3):
        return self.latest_error is not None and \
               (self.np_lb <= low_eq).min()  and \
               (self.np_lbA <= low_eq).min() and \
               (self.np_ub >= up_eq).min()   and \
               (self.np_ubA >= up_eq).min()

    def get_violating_bounds(self, low_eq=1e-3, up_eq=-1e-3):
        return {n: (l, u) for n, l, u in zip(self.row_names, 
                                             self.np_lbA.flatten(),
                                             self.np_ubA.flatten()) if l > low_eq or u < up_eq}

    def last_matrix_str(self):
        if len(self.A_dfs) > 0:
            return str(self.A_dfs[-1])
        return ''

    def _post_process_matrices(self, deltaT):
        pass

    @property
    def cmd_df(self):
        return pd.DataFrame(self._cmd_log, columns=[str(s) for s in self.cv])


class TypedQPBuilder(MinimalQPBuilder):
    def __init__(self, hard_constraints, soft_constraints, controlled_values):
        super(TypedQPBuilder, self).__init__(hard_constraints, soft_constraints, controlled_values)
        self.cv      = [c for c in self.cv]
        self.cv_type = [get_symbol_type(c) for c in self.cv]

    def get_command_signature(self):
        return dict(zip(self.cv, self.cv_type))


class PID_Constraint(object):
    def __init__(self, error_term, control_value, weight=1, k_p=1, k_i=0, k_d=0):
        self.error_term    = error_term
        self.control_value = control_value
        self.weight = weight
        self.k_p    = k_p
        self.k_i    = k_i
        self.k_d    = k_d

    @property
    def free_symbols(self):
        out = set()
        if hasattr(self.error_term, 'free_symbols'):
            out.update(self.error_term.free_symbols)
        if hasattr(self.control_value, 'free_symbols'):
            out.update(self.control_value.free_symbols) 
        return out

    def to_soft_constraint(self):
        return SoftConstraint(-self.error_term, -self.error_term, self.weight, self.control_value)

    def to_hard_constraint(self):
        return Constraint(-self.error_term, -self.error_term, self.control_value)

    def to_constraint(self):
        return self.to_hard_constraint() if self.weight is None else self.to_soft_constraint()

    def __eq__(self, other):
        if isinstance(other, PID_Constraint):
            return self.error_term == other.error_term and self.control_value == other.control_value and self.weight == other.weight and self.k_p == other.k_p and self.k_i == other.k_i and self.k_d == other.k_d
        return False

    def __str__(self):
        return 'Error term: {}\nControl term: {}\nWeight: {}\nP: {} I: {} D: {}'.format(str(self.error_term), str(self.control_value), str(self.weight), self.k_p, self.k_i, self.k_d)


class PIDQPBuilder(TypedQPBuilder):
    def __init__(self, hard_constraints, soft_constraints, controlled_values):
        pid_factors = []

        for k in hard_constraints:
            c = hard_constraints[k]
            if isinstance(c, PID_Constraint):
                hard_constraints[k] = c.to_hard_constraint()
                pid_factors.append((c.k_p, c.k_i, c.k_d))
            else:
                pid_factors.append((1, 0, 0))

        for k in soft_constraints:
            c = soft_constraints[k]
            if isinstance(c, PID_Constraint):
                soft_constraints[k] = c.to_soft_constraint()
                pid_factors.append((c.k_p, c.k_i, c.k_d))
            else:
                pid_factors.append((1, 0, 0))

        self.pid_factors = np.array(pid_factors).T
        self._old_lbA = np.zeros(len(pid_factors))
        self._old_ubA = np.zeros(len(pid_factors))
        self._int_lbA = np.zeros(len(pid_factors))
        self._int_ubA = np.zeros(len(pid_factors))

        super(PIDQPBuilder, self).__init__(hard_constraints, soft_constraints, controlled_values)


    def _post_process_matrices(self, deltaT):
        if deltaT is None:
            raise Exception('PIDQPBuilder needs valid deltaT, "{}" was given.'.format(deltaT))

        true_lbA = self.np_lbA.copy()
        true_ubA = self.np_ubA.copy()        

        self.np_lbA = np.sum(np.vstack((self.np_lbA, self._int_lbA, (self.np_lbA - self._old_lbA) / deltaT)) * self.pid_factors, axis=0) #, keepdims=True)
        self.np_ubA = np.sum(np.vstack((self.np_ubA, self._int_ubA, (self.np_ubA - self._old_ubA) / deltaT)) * self.pid_factors, axis=0) #, keepdims=True)

        self._old_lbA  = true_lbA 
        self._old_ubA  = true_ubA
        self._int_lbA += true_lbA
        self._int_ubA += true_ubA


class GeomQPBuilder(PIDQPBuilder): #(TypedQPBuilder):
    @profile
    def __init__(self, collision_world, hard_constraints, soft_constraints, controlled_values, default_query_distance=10.0, visualizer=None):

        if visualizer is not None and not isinstance(visualizer, ROSBPBVisualizer):
            raise Exception('Visualizer needs to be an instance of ROSBPBVisualizer. Given argument is of type {}'.format(type(visualizer)))

        self.visualizer = visualizer

        self.collision_handlers = {}
        symbols = set()
        for c in list(hard_constraints.values()) + list(soft_constraints.values()):
            symbols |= c.free_symbols

        self.name_resolver = {}

        #print('\n'.join(collision_world.names))

        missing_objects = set()

        for s in symbols:
            s_str = str(s)
            if s_str[:9] == 'contact__':
                path = Path(s)
                #print(path)
                obj_a = path[1:path.index(obj_to_obj_infix)] # Cut of the 'contact' and the onA/x bits
                if obj_a not in collision_world.named_objects:
                    print('Object "{}" was referenced in symbol "{}" but is not part of the collision world. Skipping it.'.format(obj_a, s_str))
                    missing_objects.add(obj_a)
                    continue

                coll_a = collision_world.named_objects[obj_a]
                self.name_resolver[coll_a] = obj_a

                obj_b = path[path.index(obj_to_obj_infix) + 1: -2]
                #print('Adding handler for distance\n {} ->\n {}'.format(obj_a_str, obj_b_str))
                if obj_b[0] == 'anon':
                    n_anon = 0
                    if len(obj_b) > 1:
                        n_anon = int(obj_b[1]) + 1 
                    if coll_a not in self.collision_handlers:
                        self.collision_handlers[coll_a] = ContactHandler(obj_a)
                    handler = self.collision_handlers[coll_a]
                    if handler.num_anon_contacts < n_anon:
                        for x in range(n_anon - handler.num_anon_contacts):
                            handler.add_passive_handle()
                
                else: 
                    if obj_b not in collision_world.named_objects:
                        print('Object "{}" was referenced in symbol "{}" but is not part of the collision world. Skipping it.'.format(obj_b, s_str))
                        missing_objects.add(obj_b)
                        continue

                    coll_b = collision_world.named_objects[obj_b]
                    self.name_resolver[coll_b] = obj_b

                    if coll_b not in self.collision_handlers:
                        if coll_a not in self.collision_handlers:
                            self.collision_handlers[coll_a] = ContactHandler(obj_a)
                        handler = self.collision_handlers[coll_a]
                        if not handler.has_handle_for(obj_b):
                            handler.add_active_handle(obj_b)
                    else:
                        handler = self.collision_handlers[coll_b]
                        if not handler.has_handle_for(obj_a):
                            handler.add_active_handle(obj_a)

        if len(missing_objects) > 0:
            raise Exception('Missing objects to compute queries:\n  {}\nObjects in world:\n  {}'.format('\n  '.join(sorted(missing_objects)), '\n  '.join(sorted(str(n) for n in collision_world.names))))

        self.closest_query_batch = {collision_object: default_query_distance for collision_object in self.collision_handlers.keys()}
        self._cb_draw = None

        self.collision_world = collision_world
        super(GeomQPBuilder, self).__init__(hard_constraints, soft_constraints, controlled_values)


    @profile
    def compute_queries(self, substitutions):
        self.collision_world.update_world(substitutions)
        closest = self.collision_world.closest_distances(self.closest_query_batch)

        if self.visualizer is not None:
            self.visualizer.begin_draw_cycle('debug_world', 'debug_contacts')
            self.visualizer.draw_world('debug_world', self.collision_world.world)

        for obj, contacts in closest.items():
            if obj in self.collision_handlers:
                handler = self.collision_handlers[obj]
                handler.handle_contacts(contacts, self.name_resolver)
                if self.visualizer is not None:
                    filtered_contacts = [cp for cp in contacts if cp.obj_b in self.name_resolver and self.name_resolver[cp.obj_b] in handler.var_map]
                    self.visualizer.draw_contacts('debug_contacts', filtered_contacts, 0.01)
                substitutions.update(handler.state)
            else:
                 print('A result for collision object {} was returned, even though this object has no handler.'.format(obj))

        if self.visualizer is not None:
            self.visualizer.render('debug_world', 'debug_contacts')

    @profile
    def get_cmd(self, substitutions, nWSR=None, deltaT=None):
        self.compute_queries(substitutions)

        cmd = super(GeomQPBuilder, self).get_cmd(substitutions, nWSR, deltaT)
        if self.visualizer is not None and self._cb_draw is not None:
            self._cb_draw(self.visualizer, substitutions, cmd)
        return cmd


@profile
def generate_controlled_values(constraints, symbols, weights={}, bounds={}, default_weight=0.01, default_bounds=(-1e9, 1e9)):
    controlled_values = {}
    to_remove  = set()

    for k, c in constraints.items():
        if type(c) == PID_Constraint:
            c = c.to_constraint()

        if cm.is_symbol(c.expr) and c.expr in symbols and str(c.expr) not in controlled_values:
            weight = default_weight if c.expr not in weights else weights[c.expr] 
            controlled_values[str(c.expr)] = ControlledValue(c.lower, c.upper, c.expr, weight)
            to_remove.add(k)

    new_constraints = {k: c for k, c in constraints.items() if k not in to_remove}
    for s in symbols:
        if str(s) not in controlled_values:
            lower, upper = default_bounds if s not in bounds else bounds[s]
            weight = default_weight if s not in weights else weights[s]
            controlled_values[str(s)] = ControlledValue(lower, upper, s, weight)

    return controlled_values, new_constraints

def depth_weight_controlled_values(km, controlled_values, default_weight=0.01, exp_factor=1.0):
    for cv in controlled_values.values():
        cv.weight_id = default_weight * max(1, len(km.get_active_geometry_raw({cm.IntSymbol(cv.symbol)}))) ** exp_factor
    return controlled_values

def find_constant_bounds(constraints):
    mins  = {}
    maxes = {}

    for c in constraints.values():
        if cm.is_symbol(c.expr):
            if not is_symbolic(c.lower):
                mins[c.expr] = c.lower if c.expr not in mins  else max(mins[c.expr], c.lower)

            if not is_symbolic(c.upper):
                mins[c.expr] = c.upper if c.expr not in maxes else max(mins[c.expr], c.upper)

    return {s: (mins[s] if s in mins else None, maxes[s] if s in maxes else None) for s in set(mins.keys()).union(set(maxes.keys()))}
