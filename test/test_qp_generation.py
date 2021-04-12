import kineverse.gradients.common_math as cm

from kineverse.gradients.diff_logic     import TYPE_POSITION, \
                                               TYPE_VELOCITY, \
                                               TYPE_ACCEL, \
                                               TYPE_JERK, \
                                               TYPE_SNAP, \
                                               Symbol,    \
                                               Position,  \
                                               Velocity,  \
                                               Acceleration, \
                                               Jerk, \
                                               Snap, \
                                               create_symbol, \
                                               erase_type, \
                                               get_symbol_type
from kineverse.gradients.gradient_math  import se, DiffSymbol, IntSymbol, extract_expr, wrap_expr

from kineverse.model.articulation_model import Constraint
from kineverse.motion.min_qp_builder    import SoftConstraint, ControlledValue

import unittest as ut


DT_SYM = Position('dT')

def convert_constraints_to_opt(cvs, constr_dict):

    lconstr = constr_dict.items()

    lbA = []
    ubA = []

    syms   = [erase_type(cv.symbol) for cv in cvs]
    t_syms = [get_symbol_type(cv.symbol) for cv in cvs]

    t_cons = [TYPE_POSITION, TYPE_VELOCITY, TYPE_ACCEL, TYPE_JERK, TYPE_SNAP]

    rows = []

    for k, c in constr_dict.items():
        if type(c) == SoftConstraint:
            lbA.append(extract_expr(c.lower))
            ubA.append(extract_expr(c.upper))
            rows.append([c.expr[cv.symbol] for cv in cvs])
        elif isinstance(c, Constraint):
            lbA.append(extract_expr(c.lower - c.expr))
            ubA.append(extract_expr(c.upper - c.expr))


            if type(c.expr) == Symbol:
                ti = get_symbol_type(c.expr)

                for si, s in zip(t_syms, syms):
                    

            expr = wrap_expr(c.expr)

            row = []
            for si, s in zip(t_syms, syms):
                factor = 1
                
                for ti in t_cons:
                    ts = create_symbol(s, ti)
                    
                    if ts in expr.free_symbols:
                        factor = DT_SYM ** (si - ti)
                        break

                row.append(extract_expr(expr[create_symbol(s, si)] * factor))

            rows.append(row)

    return cm.Matrix(rows), cm.Matrix(lbA), cm.Matrix(ubA)



class TestEventModel(ut.TestCase):
    def test_matrix_A_generator(self):
        a, b, c = [Position(x) for x in 'abc']
        c1 = Constraint(-2, 1, a)
        c2 = Constraint( 1, 2, b)
        c3 = Constraint( 2, 4, c)
        c4 = Constraint( 2, 4, DiffSymbol(a))

        cvs = [ControlledValue(-1, 1, DiffSymbol(a)), ControlledValue(-1, 1, DiffSymbol(b)), ControlledValue(-1, 1, DiffSymbol(c))]

        A, lbA, ubA = convert_constraints_to_opt(cvs, {x: c for x, c in enumerate([c1, c2, c3, c4])})


        print(A, lbA.row_join(ubA))


if __name__ == '__main__':
    ut.main()
