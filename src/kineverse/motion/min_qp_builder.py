import giskardpy.symengine_wrappers as spw
import numpy as np

from giskardpy import BACKEND
from giskardpy.exceptions import QPSolverException
from giskardpy.qp_solver  import QPSolver

from kineverse.gradients.diff_logic         import get_symbol_type
from kineverse.gradients.gradient_container import GradientContainer as GC

default_bound = 1e9    

class HardConstraint(object):
    def __init__(self, lower, upper, expr):
        self.lower = lower
        self.upper = upper
        self.expr  = expr

class SoftConstraint(HardConstraint):
    def __init__(self, lower, upper, weight, expr):
        super(SoftConstraint, self).__init__(lower, upper, expr)
        self.weight = weight

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
        hc = [(k, HardConstraint(extract_expr(c.lower), extract_expr(c.upper), wrap_expr(c.expr))) for k, c in hard_constraints.items()]
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


    def get_cmd(self, substitutions, nWSR=None):
        np_big_ass_M = self.cython_big_ass_M(**substitutions)
        self.np_H   = np.array(np_big_ass_M[self.shape1:, :-2])
        self.np_A   = np.array(np_big_ass_M[:self.shape1, :self.shape2])
        self.np_lb  = np.array(np_big_ass_M[self.shape1:, -2])
        self.np_ub  = np.array(np_big_ass_M[self.shape1:, -1])
        self.np_lbA = np.array(np_big_ass_M[:self.shape1, -2])
        self.np_ubA = np.array(np_big_ass_M[:self.shape1, -1])
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


class TypedQPBuilder(MinimalQPBuilder):
    def __init__(self, hard_constraints, soft_constraints, controlled_values):
        super(TypedQPBuilder, self).__init__(hard_constraints, soft_constraints, controlled_values)
        self.cv      = [c for c in self.cv]
        self.cv_type = [get_symbol_type(c) for c in self.cv]

    def get_command_signature(self):
        return dict(zip(self.cv, self.cv_type))

