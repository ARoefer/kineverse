import unittest as ut
import kineverse.gradients.common_math as cm

from kineverse.gradients.diff_logic    import DiffSymbol, Position
from kineverse.gradients.gradient_math import *


class TestOperators(ut.TestCase):

    def test_addition(self):
        x, y = [Position(x) for x in 'xy']

        baseline = x + y
        gc_x = GC(x)
        gc_y = GC(y)
        
        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = gc_x + gc_y

        self.assertTrue(cm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(x)], cm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(y)], cm.diff(baseline, y), default_range=[-10, 10]))

    def test_subtraction(self):
        x, y = [Position(x) for x in 'xy']

        baseline = x - y
        gc_x = GC(x)
        gc_y = GC(y)
        
        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = gc_x - gc_y

        self.assertTrue(cm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(x)], cm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(y)], cm.diff(baseline, y), default_range=[-10, 10]))


    def test_multiplication(self):
        x, y = [Position(x) for x in 'xy']

        baseline = x * (5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)
        
        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = gc_x * gc_y

        self.assertTrue(cm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(x)], cm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(y)], cm.diff(baseline, y), default_range=[-10, 10]))


    def test_division(self):
        x, y = [Position(x) for x in 'xy']

        baseline = x / (5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)
        
        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = gc_x / gc_y

        self.assertTrue(cm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(x)], cm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(y)], cm.diff(baseline, y), default_range=[-10, 10]))


    def testow(self):
        x, y = [Position(x) for x in 'xy']

        baseline = x ** (4 * y)
        gc_x = GC(x)
        gc_y = GC(4 * y)
        
        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = gc_x ** gc_y

        temp = cm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10])
        print(gc_xy.expr, baseline)
        print(temp, type(temp))
        self.assertTrue(temp)
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(x)], cm.diff(baseline, x), default_range=[-10, 10]))
        d_grad_y = gc_xy[DiffSymbol(y)]
        d_baseline_y = cm.diff(baseline, y)
        print('Gradient d/y: {}\nBaseline d/y: {}'.format(d_grad_y, d_baseline_y))
        print(d_baseline_y)
        self.assertTrue(cm.numeric_eq(d_grad_y, d_baseline_y, default_range=[-10, 10]))


class TestFunctions(ut.TestCase):

    def test_sin(self):
        x, y = [Position(x) for x in 'xy']

        baseline = cm.sin(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = sin(gc_x + gc_y)

        self.assertTrue(cm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(x)], cm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(y)], cm.diff(baseline, y), default_range=[-10, 10]))

    def test_cos(self):
        x, y = [Position(x) for x in 'xy']

        baseline = cm.cos(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = cos(gc_x + gc_y)

        self.assertTrue(cm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(x)], cm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(y)], cm.diff(baseline, y), default_range=[-10, 10]))

    def test_tan(self):
        x, y = [Position(x) for x in 'xy']

        baseline = cm.tan(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = tan(gc_x + gc_y)

        self.assertTrue(cm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(x)], cm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(y)], cm.diff(baseline, y), default_range=[-10, 10]))

    def test_asin(self):
        x, y = [Position(x) for x in 'xy']

        baseline = cm.asin(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = asin(gc_x + gc_y)

        self.assertTrue(cm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(x)], cm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(y)], cm.diff(baseline, y), default_range=[-10, 10]))

    def test_acos(self):
        x, y = [Position(x) for x in 'xy']

        baseline = cm.acos(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = acos(gc_x + gc_y)

        self.assertTrue(cm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(x)], cm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(y)], cm.diff(baseline, y), default_range=[-10, 10]))

    def test_atan(self):
        x, y = [Position(x) for x in 'xy']

        baseline = cm.atan(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = atan(gc_x + gc_y)

        self.assertTrue(cm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(x)], cm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(y)], cm.diff(baseline, y), default_range=[-10, 10]))

    def test_sinh(self):
        x, y = [Position(x) for x in 'xy']

        baseline = cm.sinh(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = sinh(gc_x + gc_y)

        self.assertTrue(cm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(x)], cm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(y)], cm.diff(baseline, y), default_range=[-10, 10]))

    def test_cosh(self):
        x, y = [Position(x) for x in 'xy']

        baseline = cm.cosh(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = cosh(gc_x + gc_y)

        self.assertTrue(cm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(x)], cm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(y)], cm.diff(baseline, y), default_range=[-10, 10]))

    def test_tanh(self):
        x, y = [Position(x) for x in 'xy']

        baseline = cm.tanh(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = tanh(gc_x + gc_y)

        self.assertTrue(cm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(x)], cm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(y)], cm.diff(baseline, y), default_range=[-10, 10]))

    def test_asinh(self):
        x, y = [Position(x) for x in 'xy']

        baseline = cm.asinh(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = asinh(gc_x + gc_y)

        self.assertTrue(cm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(x)], cm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(y)], cm.diff(baseline, y), default_range=[-10, 10]))

    def test_acosh(self):
        x, y = [Position(x) for x in 'xy']

        baseline = cm.acosh(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = acosh(gc_x + gc_y)

        self.assertTrue(cm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(x)], cm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(y)], cm.diff(baseline, y), default_range=[-10, 10]))

    def test_atanh(self):
        x, y = [Position(x) for x in 'xy']

        baseline = cm.atanh(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = atanh(gc_x + gc_y)

        self.assertTrue(cm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(x)], cm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(y)], cm.diff(baseline, y), default_range=[-10, 10]))

    def test_exp(self):
        x, y = [Position(x) for x in 'xy']

        baseline = cm.exp(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = exp(gc_x + gc_y)

        self.assertTrue(cm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(x)], cm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(y)], cm.diff(baseline, y), default_range=[-10, 10]))

    def test_log(self):
        x, y = [Position(x) for x in 'xy']

        baseline = cm.log(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = log(gc_x + gc_y)

        self.assertTrue(cm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(x)], cm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(y)], cm.diff(baseline, y), default_range=[-10, 10]))

    def test_sqrt(self):
        x, y = [Position(x) for x in 'xy']

        baseline = cm.sqrt(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = sqrt(gc_x + gc_y)

        self.assertTrue(cm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(x)], cm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(y)], cm.diff(baseline, y), default_range=[-10, 10]))

    def test_abs(self):
        x, y = [Position(x) for x in 'xy']

        baseline = abs(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = abs(gc_x + gc_y)

        self.assertTrue(cm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(x)], cm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gc_xy[DiffSymbol(y)], cm.diff(baseline, y), default_range=[-10, 10]))


class TestMatrix(ut.TestCase):
    def test_operators(self):
        M = cm.Matrix([[1,2],[3,4],[5,6]])

        baseline = cm.dot(M, cm.Matrix([[4],[9]]))
        gm_M = GM(M)
        gm_R = cm.dot(gm_M, cm.Matrix([[4],[9]]))

        self.assertTrue(cm.numeric_eq(gm_M.to_sym_matrix(), M, default_range=[-10, 10]))
        self.assertTrue(cm.numeric_eq(gm_R.to_sym_matrix(), baseline, default_range=[-10, 10]))

    def test_transpose(self):
        M = cm.Matrix([[1,2],[3,4],[5,6]])

        baseline = M.T
        gm_M = GM(M).T

        for e, x in zip(gm_M, GM(baseline)):
            self.assertTrue(cm.numeric_eq(e.expr, x.expr, default_range=[-10, 10]))


if __name__ == '__main__':
    ut.main()
