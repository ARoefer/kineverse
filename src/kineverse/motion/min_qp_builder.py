import giskardpy.symengine_wrappers as spw
import numpy as np

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

class ControlledValue(object):
    def __init__(self, lower, upper, symbol, weight=1):
        self.lower  = lower
        self.upper  = upper
        self.symbol = symbol
        self.weight = weight

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

        self.big_ass_M = self.A.row_join(self.lbA).row_join(self.ubA).col_join(self.H.row_join(self.lb).row_join(self.ub))

        self.free_symbols = self.big_ass_M.free_symbols
        self.cython_big_ass_M = spw.speed_up(self.big_ass_M, self.free_symbols, backend=BACKEND)

        self.cv        = [c.symbol for _, c in cv]
        self.n_cv      = len(cv)
        self.row_names = [k for k, _ in hc + sc]
        self.col_names = [k for k, _ in cv + sc]
        self.np_col_header = np.array([''] + self.col_names).reshape((1, len(self.col_names) + 1))
        self.np_row_header = np.array(self.row_names).reshape((len(self.row_names), 1))

        self.shape1    = len(self.row_names)
        self.shape2    = len(self.col_names)
        self.qp_solver = QPSolver(len(self.col_names), len(self.row_names))

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

        try:
            xdot_full = self.qp_solver.solve(self.np_H, self.np_g, self.np_A, self.np_lb,  self.np_ub, 
                                                                    self.np_lbA, self.np_ubA, nWSR)
        except QPSolverException as e:
            print('INFEASIBLE CONFIGURATION!\n{}'.format(self._create_display_string(self.np_H, self.np_A, self.np_lb, self.np_ub, self.np_lbA, self.np_ubA)))
            raise e
        if xdot_full is None:
            return None

        return {cv: xdot_full[i] for i, cv in enumerate(self.cv)}

    def equilibrium_reached(self, low_eq=1e-3, up_eq=-1e-3):
        return (self.np_lb <= low_eq).min() and (self.np_lbA <= low_eq).min() and (self.np_ub >= up_eq).min() and (self.np_ubA >= up_eq).min()

    def last_matrix_str(self):
        return self._create_display_string(self.np_H, self.np_A, self.np_lb, self.np_ub, self.np_lbA, self.np_ubA)

    def _create_display_string(self, np_H, np_A, np_lb, np_ub, np_lbA, np_ubA):
        h_str  = np.array_str(np.vstack((self.np_col_header, np.hstack((self.np_col_header.T[1:], np_H)))), precision=4)
        a_str  = np.array_str(np.vstack((self.np_col_header[:, :self.n_cv + 1], np.hstack((self.np_row_header, np_A[:, :self.n_cv])))), precision=4)
        b_str  = np.array_str(np.vstack((np.array([['', 'lb', 'ub']]), np.hstack((self.np_col_header.T[1:], np_lb.reshape((np_lb.shape[0], 1)), np_ub.reshape((np_ub.shape[0], 1)))))))
        bA_str = np.array_str(np.vstack((np.array([['', 'lbA', 'ubA']]), np.hstack((self.np_row_header, np_lbA.reshape((np_lbA.shape[0], 1)), np_ubA.reshape((np_ubA.shape[0], 1)))))))

        return 'H =\n{}\nlb, ub =\n{}\nA =\n{}\nlbA, ubA =\n{}\n'.format(h_str, b_str, a_str, bA_str)

    def _post_process_matrices(self, deltaT):
        pass


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
            self.visualizer.begin_draw_cycle()
            self.visualizer.draw_world('debug', self.collision_world)
            #for contacts in closest.values():
            #    self.visualizer.draw_contacts('debug', contacts, 0.05)
            self.visualizer.render()

        for obj, contacts in closest.items():
            if obj in self.collision_handlers:
                handler = self.collision_handlers[obj]
                handler.handle_contacts(contacts, self.name_resolver)
                substitutions.update(handler.state)
            # else:
            #     raise Exception('A result for collision object {} was returned, even though this object has no handler.'.format(obj))

        return super(GeomQPBuilder, self).get_cmd(substitutions, nWSR, deltaT)


        




