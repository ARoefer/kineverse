import giskardpy.symengine_wrappers as spw
import numpy  as np
import pandas as pd

from giskardpy import BACKEND
from giskardpy.exceptions import QPSolverException
from giskardpy.qp_solver  import QPSolver

from kineverse.gradients.diff_logic         import get_symbol_type
from kineverse.gradients.gradient_container import GradientContainer as GC
from kineverse.model.kinematic_model        import Constraint, Path
from kineverse.model.geometry_model         import CollisionSubworld, ContactHandler, obj_to_obj_infix
from kineverse.visualization.bpb_visualizer import ROSBPBVisualizer

default_bound = 1e9    

HardConstraint = Constraint

class SoftConstraint(Constraint):
    def __init__(self, lower, upper, weight, expr):
        super(SoftConstraint, self).__init__(lower, upper, expr)
        self.weight = weight

    @classmethod
    def from_constraint(cls, constraint, weight=1):
        return cls(constraint.lower, constraint.upper, weight, constraint.expr)

    def __str__(self):
        return '{} @ {}'.format(super(SoftConstraint, self).__str__(), self.weight)

class ControlledValue(object):
    def __init__(self, lower, upper, symbol, weight=1):
        self.lower  = lower
        self.upper  = upper
        self.symbol = symbol
        self.weight = weight

    def __str__(self):
        return '{} <= {} <= {} @ {}'.format(self.lower, self.symbol, self.upper, self.weight)

def extract_expr(expr):
    return expr if type(expr) != GC else expr.expr

def wrap_expr(expr):
    return expr if type(expr) == GC else GC(expr)

class MinimalQPBuilder(object):
    def __init__(self, hard_constraints, soft_constraints, controlled_values):
        hc = [(k, Constraint(extract_expr(c.lower), extract_expr(c.upper), wrap_expr(c.expr))) for k, c in hard_constraints.items()]
        sc = [(k, SoftConstraint(extract_expr(c.lower), extract_expr(c.upper), c.weight, wrap_expr(c.expr))) for k, c in soft_constraints.items()]
        cv = [(k, ControlledValue(extract_expr(c.lower), extract_expr(c.upper), c.symbol, extract_expr(c.weight))) for k, c in controlled_values.items()]

        self.np_g = np.zeros(len(cv + sc))
        self.H    = spw.diag(*[c.weight for _, c in cv + sc])
        self.lb   = spw.Matrix([c.lower if c.lower is not None else -default_bound for _, c in cv] + [-default_bound] * len(sc))
        self.ub   = spw.Matrix([c.upper if c.upper is not None else  default_bound for _, c in cv] + [default_bound] * len(sc))
        self.lbA  = spw.Matrix([c.lower if c.lower is not None else -default_bound for _, c in hc + sc])
        self.ubA  = spw.Matrix([c.upper if c.upper is not None else  default_bound for _, c in hc + sc])

        M_cv      = [c.symbol for _, c in cv]
        self.A    = spw.Matrix([[c.expr[s] if s in c.expr else 0 for s in M_cv] for _, c in hc + sc]).row_join(spw.zeros(len(hc), len(sc)).col_join(spw.eye(len(sc))))

        self.cv        = [c.symbol for _, c in cv]
        self.n_cv      = len(cv)
        self.n_hc      = len(hc)
        self.n_sc      = len(sc)
        self.row_names = [k for k, _ in hc + sc]
        self.col_names = [k for k, _ in cv + sc]
        self.A_dfs     = []
        self.H_dfs     = []
        self._cmd_log  = []
        
        self._build_M()


    # Add new constraints as you go
    def add_constraints(self, hard_constraints, soft_constraints):
        if len(hard_constraints) > 0 or len(soft_constraints) > 0:
            hc = [(k, Constraint(extract_expr(c.lower), extract_expr(c.upper), wrap_expr(c.expr))) for k, c in hard_constraints.items()]
            sc = [(k, SoftConstraint(extract_expr(c.lower), extract_expr(c.upper), c.weight, wrap_expr(c.expr))) for k, c in soft_constraints.items()]

            A_addition   = spw.zeros(1, self.A.shape[1])
            lbA_addition = spw.zeros(1)
            ubA_addition = spw.zeros(1)
            new_weights  = []

            for k, c in hc + sc:
                try:
                    idx = self.row_names.index(k)
                    is_hc = max(self.A[idx, len(self.cv):]) == 0
                    if type(c) is SoftConstraint and is_hc:
                        raise Exception('key "{}" refers to hard constraint but is being overriden with a soft constraint.')
                    elif idx >= self.n_hc:
                        raise Exception('key "{}" refers to soft constraint but is being overriden with a hard constraint.')

                    self.lbA[idx] = c.lower
                    self.ubA[idx] = c.upper
                    for x, s in enumerate(self.cv):
                            self.A[idx, x] = c.expr[s] if s in c.expr else 0
                    if type(c) is SoftConstraint:
                        d_idx = self.n_cv + idx - self.n_hc
                        self.H[d_idx, d_idx] = c.weight
                except ValueError:
                    lbA_addition = lbA_addition.col_join(spw.Matrix([c.lower]))
                    ubA_addition = ubA_addition.col_join(spw.Matrix([c.upper]))
                    if type(c) is SoftConstraint:
                        A_addition = A_addition.row_join(spw.zeros(A_addition.shape[0], 1).col_join(spw.Matrix([1])))
                        new_weights.append(c.weight)
                        self.col_names.append(k)

                    self.row_names.append(k)
                    A_addition = A_addition.col_join(spw.Matrix([c.expr[s] if s in c.expr else 0 for s in self.cv] + [0] * (A_addition.shape[1] - self.n_cv)))

            if A_addition.shape[1] > 1: # New constraints have been added
                self.lbA = self.lbA.col_join(lbA_addition[1:,:])
                self.ubA = self.ubA.col_join(ubA_addition[1:,:])
                if A_addition.shape[1] > self.A.shape[1]:
                    self.A = self.A.row_join(spw.zeros(self.A.shape[0], A_addition.shape[1] - self.A.shape[1]))
                self.A   = self.A.col_join(A_addition[1:,:])

            if len(new_weights) > 0:
                self.H = self.H.row_join(spw.zeros(self.H.shape[0], len(new_weights))).col_join(spw.zeros(len(new_weights), self.H.shape[1]).row_join(spw.diag(*new_weights)))

            self.n_hc += len(lbA_addition) - 1 - len(new_weights)
            self.n_sc += len(new_weights)

            self._build_M()


    def remove_constraints(self, constraints):
        indices = {n: x for x, n in enumerate(self.row_names)}

        to_remove = {indices[k] for k in constraints if k in indices}
        if len(to_remove) > 0:
            new_A_rows   = []
            new_lbA      = []
            new_ubA      = []
            new_weights  = [self.H[x, x] for x in range(self.H.shape[0] - self.n_sc)]
            sc_idx = 0 
            for x in range(self.A.shape[0]):
                if x not in to_remove:
                    new_lbA.append(self.lbA[x,:])
                    new_ubA.append(self.lbA[x,:])
                    if max(self.A[indices[k], len(self.cv):] != 0): # this is a soft constraint
                        new_weights.append(self.H[len(self.cv) + sc_idx, len(self.cv) + sc_idx])
                        new_A_rows.append(self.A[x, :].row_join(spw.Matrix(([0] * sc_idx) + [1])))
                        sc_idx += 1
                    else:
                        new_A_rows.append(self.A[x, :])
                else:
                    del self.row_names[x]

            self.n_sc = sc_idx
            self.n_hc = len(new_A_rows) - self.n_sc

            self.A   = spw.Matrix([row.row_join(spw.Matrix([0] * (sc_idx + len(self.cv) - row.shape[1]))) for row in new_A_rows])
            self.H   = spw.diag(*new_weights)
            self.lbA = spw.Matrix(new_lbA)
            self.ubA = spw.Matrix(new_ubA)


    def _build_M(self):
        self.big_ass_M = self.A.row_join(self.lbA).row_join(self.ubA).col_join(self.H.row_join(self.lb).row_join(self.ub))

        self.free_symbols     = self.big_ass_M.free_symbols
        self.cython_big_ass_M = spw.speed_up(self.big_ass_M, self.free_symbols, backend=BACKEND)

        self.shape1    = self.A.shape[0]
        self.shape2    = self.A.shape[1]

        self.reset_solver()

    def reset_solver(self):
        self.qp_solver = QPSolver(self.A.shape[1], self.A.shape[0])

    @profile
    def get_cmd(self, substitutions, nWSR=None, deltaT=None):
        substitutions = {str(s): v for s, v in substitutions.items()}
        np_big_ass_M = self.cython_big_ass_M(**substitutions)
        self.np_H   = np.array(np_big_ass_M[self.shape1:, :-2])
        self.np_A   = np.array(np_big_ass_M[:self.shape1, :self.shape2])
        self.np_lb  = np.array(np_big_ass_M[self.shape1:, -2])
        self.np_ub  = np.array(np_big_ass_M[self.shape1:, -1])
        self.np_lbA = np.array(np_big_ass_M[:self.shape1, -2])
        self.np_ubA = np.array(np_big_ass_M[:self.shape1, -1])

        self._post_process_matrices(deltaT)

        dfH = pd.DataFrame(np.vstack((self.np_lb, self.np_ub, self.np_H.diagonal())), index=['lb', 'ub', 'weight'], columns=self.col_names)
        dfA = pd.DataFrame(np.hstack((self.np_lbA.reshape((self.shape1, 1)), 
                                      self.np_ubA.reshape((self.shape1, 1)), 
                                      self.np_A[:, :-self.n_sc])), 
                                      index=self.row_names, columns=['lbA', 'ubA'] + self.col_names[:self.n_cv])

        try:
            xdot_full = self.qp_solver.solve(self.np_H, self.np_g, self.np_A, self.np_lb,  self.np_ub, 
                                                                    self.np_lbA, self.np_ubA, nWSR)
            self.A_dfs.append(dfA)
            self.H_dfs.append(dfH)
            self._cmd_log.append(xdot_full[:self.n_cv])
        except QPSolverException as e:
            print('INFEASIBLE CONFIGURATION!\nH:{}\nA:\n{}'.format(dfH, dfA))
            b_comp  = np.greater(self.np_lb,  self.np_ub)
            bA_comp = np.greater(self.np_lbA, self.np_ubA)
            if b_comp.max() or bA_comp.max():
                print('Overlapping boundary conditions:\n{}\n{}'.format('\n'.join(['{}:\n  lb: {}\n  ub: {}'.format(n, lb, ub) for n, lb, ub, c in zip(self.col_names, self.np_lb, self.np_ub, b_comp) if c]),
                     '\n'.join(['{}:\n  lbA: {}\n  ubA: {}'.format(n, lbA, ubA) for n, lbA, ubA, c in zip(self.row_names, self.np_lbA, self.np_ubA, bA_comp) if c])))
            else:
                print('Boundaries are not overlapping. Error must be more complex.')

            raise e
        if xdot_full is None:
            return None

        return {cv: xdot_full[i] for i, cv in enumerate(self.cv)}

    def equilibrium_reached(self, low_eq=1e-3, up_eq=-1e-3):
        return (self.np_lb <= low_eq).min() and (self.np_lbA <= low_eq).min() and (self.np_ub >= up_eq).min() and (self.np_ubA >= up_eq).min()

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


class PID_Constraint():
    def __init__(self, error_term, control_value, weight=1, k_p=1, k_i=0, k_d=0):
        self.error_term    = error_term
        self.control_value = control_value
        self.weight = weight
        self.k_p    = k_p
        self.k_i    = k_i
        self.k_d    = k_d

    def to_soft_constraint(self):
        return SoftConstraint(-self.error_term, -self.error_term, self.weight, self.control_value)

    def to_hard_constraint(self):
        return Constraint(-self.error_term, -self.error_term, self.control_value)

    def to_constraint(self):
        return self.to_hard_constraint() if self.weight is None else self.to_soft_constraint()


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

        self.pid_factors = np.array(pid_factors)
        self._old_lbA = np.zeros((len(pid_factors), 1))
        self._old_ubA = np.zeros((len(pid_factors), 1))
        self._int_lbA = np.zeros((len(pid_factors), 1))
        self._int_ubA = np.zeros((len(pid_factors), 1))

        super(PIDQPBuilder, self).__init__(hard_constraints, soft_constraints, controlled_values)


    def _post_process_matrices(self, deltaT):
        if deltaT is None:
            raise Exception('PIDQPBuilder needs valid deltaT, "{}" was given.'.format(deltaT))

        true_lbA = self.np_lbA.copy()
        true_ubA = self.np_ubA.copy()        

        self.np_lbA = np.sum(np.hstack(self.np_lbA, self._int_lbA, (self.np_lbA - self._old_lbA) / deltaT) * self.pid_factors, axis=1, keepdims=True)
        self.np_ubA = np.sum(np.hstack(self.np_ubA, self._int_ubA, (self.np_ubA - self._old_ubA) / deltaT) * self.pid_factors, axis=1, keepdims=True)

        self._old_lbA  = true_lbA 
        self._old_ubA  = true_ubA
        self._int_lbA += true_lbA
        self._int_ubA += true_ubA


class GeomQPBuilder(TypedQPBuilder):
    @profile
    def __init__(self, collision_world, hard_constraints, soft_constraints, controlled_values, default_query_distance=1.0, visualizer=None):

        if visualizer is not None and not isinstance(visualizer, ROSBPBVisualizer):
            raise Exception('Visualizer needs to be an instance of ROSBPBVisualizer. Given argument is of type {}'.format(type(visualizer)))

        self.visualizer = visualizer

        self.collision_handlers = {}
        symbols = set()
        for c in hard_constraints.values() + soft_constraints.values():
            symbols |= c.free_symbols

        self.name_resolver = {}

        #print('\n'.join(collision_world.names))

        for s in symbols:
            s_str = str(s)
            if s_str[:9] == 'contact__':
                path = Path(s)
                #print(path)
                obj_a = path[1:path.index(obj_to_obj_infix)] # Cut of the 'contact' and the onA/x bits
                obj_a_str = str(obj_a)
                if obj_a_str not in collision_world.named_objects:
                    print('Object "{}" was referenced in symbol "{}" but is not part of the collision world. Skipping it.'.format(obj_a_str, s_str))
                    continue

                coll_a = collision_world.named_objects[obj_a_str]
                self.name_resolver[coll_a] = obj_a

                obj_b = path[path.index(obj_to_obj_infix) + 1: -2]
                obj_b_str = str(obj_b)
                #print('Adding handler for distance\n {} ->\n {}'.format(obj_a_str, obj_b_str))
                if obj_b[0] == 'anon':
                    if len(obj_b) > 1:
                        n_anon = int(obj_b[1]) + 1 
                    if coll_a not in self.collision_handlers:
                        self.collision_handlers[coll_a] = ContactHandler(obj_a)
                    handler = self.collision_handlers[coll_a]
                    if handler.num_anon_contacts < n_anon:
                        for x in range(n_anon - handler.num_anon_contacts):
                            handler.add_passive_handle()
                
                else: 
                    if obj_b_str not in collision_world.named_objects:
                        print('Object "{}" was referenced in symbol "{}" but is not part of the collision world. Skipping it.'.format(obj_b_str, s_str))
                        continue

                    coll_b = collision_world.named_objects[obj_b_str]
                    self.name_resolver[coll_b] = obj_b

                    if coll_b not in self.collision_handlers:
                        if obj_a_str not in self.collision_handlers:
                            self.collision_handlers[coll_b] = ContactHandler(obj_a)
                        handler = self.collision_handlers[coll_b]
                        if not handler.has_handle_for(obj_b):
                            handler.add_active_handle(obj_b)
                    else:
                        handler = self.collision_handlers[coll_b]
                        if not handler.has_handle_for(obj_a):
                            handler.add_active_handle(obj_a)
        

        self.closest_query_batch = {collision_object: default_query_distance for collision_object in self.collision_handlers.keys()}

        self.collision_world = collision_world
        super(GeomQPBuilder, self).__init__(hard_constraints, soft_constraints, controlled_values)


    @profile
    def get_cmd(self, substitutions, nWSR=None, deltaT=None):
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

        return super(GeomQPBuilder, self).get_cmd(substitutions, nWSR, deltaT)


        




