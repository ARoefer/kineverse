import unittest as ut

import kineverse.gradients.common_math   as cm
import kineverse.gradients.gradient_math as gm

from kineverse.gradients.diff_logic    import DiffSymbol, Position


class TestOperators(ut.TestCase):

    def test_addition(self):
        x, y = [gm.Position(x) for x in 'xy']

        baseline = x + y
        gc_x = gm.GC(x)
        gc_y = gm.GC(y)
        
        # Generate gradients
        gc_x[gm.DiffSymbol(x)]
        gc_y[gm.DiffSymbol(y)]

        gc_xy = gc_x + gc_y

        self.assertTrue(gm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(x)], gm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(y)], gm.diff(baseline, y), default_range=[-10, 10]))

    def test_subtraction(self):
        x, y = [gm.Position(x) for x in 'xy']

        baseline = x - y
        gc_x = gm.GC(x)
        gc_y = gm.GC(y)
        
        # Generate gradients
        gc_x[gm.DiffSymbol(x)]
        gc_y[gm.DiffSymbol(y)]

        gc_xy = gc_x - gc_y

        self.assertTrue(gm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(x)], gm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(y)], gm.diff(baseline, y), default_range=[-10, 10]))


    def test_multiplication(self):
        x, y = [gm.Position(x) for x in 'xy']

        baseline = x * (5 * y)
        gc_x = gm.GC(x)
        gc_y = gm.GC(5 * y)
        
        # Generate gradients
        gc_x[gm.DiffSymbol(x)]
        gc_y[gm.DiffSymbol(y)]

        gc_xy = gc_x * gc_y

        self.assertTrue(gm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(x)], gm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(y)], gm.diff(baseline, y), default_range=[-10, 10]))


    def test_division(self):
        x, y = [gm.Position(x) for x in 'xy']

        baseline = x / (5 * y)
        gc_x = gm.GC(x)
        gc_y = gm.GC(5 * y)
        
        # Generate gradients
        gc_x[gm.DiffSymbol(x)]
        gc_y[gm.DiffSymbol(y)]

        gc_xy = gc_x / gc_y

        self.assertTrue(gm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(x)], gm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(y)], gm.diff(baseline, y), default_range=[-10, 10]))


    def test_pow(self):
        x, y = [gm.Position(x) for x in 'xy']

        baseline = x ** (4 * y)
        gc_x = gm.GC(x)
        gc_y = gm.GC(4 * y)
        
        # Generate gradients
        gc_x[gm.DiffSymbol(x)]
        gc_y[gm.DiffSymbol(y)]

        gc_xy = gc_x ** gc_y

        temp = gm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10])
        self.assertTrue(temp)
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(x)], gm.diff(baseline, x), default_range=[-10, 10]))
        d_grad_y = gc_xy[gm.DiffSymbol(y)]
        d_baseline_y = gm.diff(baseline, y)
        self.assertTrue(gm.numeric_eq(d_grad_y, d_baseline_y, default_range=[-10, 10]))


class TestFunctions(ut.TestCase):

    def test_sin(self):
        x, y = [gm.Position(x) for x in 'xy']

        baseline = cm.sin(x + 5 * y)
        gc_x = gm.GC(x)
        gc_y = gm.GC(5 * y)

        # Generate gradients
        gc_x[gm.DiffSymbol(x)]
        gc_y[gm.DiffSymbol(y)]

        gc_xy = gm.sin(gc_x + gc_y)

        self.assertTrue(gm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(x)], gm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(y)], gm.diff(baseline, y), default_range=[-10, 10]))

    def test_cos(self):
        x, y = [gm.Position(x) for x in 'xy']

        baseline = cm.cos(x + 5 * y)
        gc_x = gm.GC(x)
        gc_y = gm.GC(5 * y)

        # Generate gradients
        gc_x[gm.DiffSymbol(x)]
        gc_y[gm.DiffSymbol(y)]

        gc_xy = gm.cos(gc_x + gc_y)

        self.assertTrue(gm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(x)], gm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(y)], gm.diff(baseline, y), default_range=[-10, 10]))

    def test_tan(self):
        x, y = [gm.Position(x) for x in 'xy']

        baseline = cm.tan(x + 5 * y)
        gc_x = gm.GC(x)
        gc_y = gm.GC(5 * y)

        # Generate gradients
        gc_x[gm.DiffSymbol(x)]
        gc_y[gm.DiffSymbol(y)]

        gc_xy = gm.tan(gc_x + gc_y)

        self.assertTrue(gm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(x)], gm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(y)], gm.diff(baseline, y), default_range=[-10, 10]))

    def test_asin(self):
        x, y = [gm.Position(x) for x in 'xy']

        baseline = cm.asin(x + 5 * y)
        gc_x = gm.GC(x)
        gc_y = gm.GC(5 * y)

        # Generate gradients
        gc_x[gm.DiffSymbol(x)]
        gc_y[gm.DiffSymbol(y)]

        gc_xy = gm.asin(gc_x + gc_y)

        self.assertTrue(gm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(x)], gm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(y)], gm.diff(baseline, y), default_range=[-10, 10]))

    def test_acos(self):
        x, y = [gm.Position(x) for x in 'xy']

        baseline = cm.acos(x + 5 * y)
        gc_x = gm.GC(x)
        gc_y = gm.GC(5 * y)

        # Generate gradients
        gc_x[gm.DiffSymbol(x)]
        gc_y[gm.DiffSymbol(y)]

        gc_xy = gm.acos(gc_x + gc_y)

        self.assertTrue(gm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(x)], gm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(y)], gm.diff(baseline, y), default_range=[-10, 10]))

    def test_atan(self):
        x, y = [gm.Position(x) for x in 'xy']

        baseline = cm.atan(x + 5 * y)
        gc_x = gm.GC(x)
        gc_y = gm.GC(5 * y)

        # Generate gradients
        gc_x[gm.DiffSymbol(x)]
        gc_y[gm.DiffSymbol(y)]

        gc_xy = gm.atan(gc_x + gc_y)

        self.assertTrue(gm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(x)], gm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(y)], gm.diff(baseline, y), default_range=[-10, 10]))

    def test_sinh(self):
        x, y = [gm.Position(x) for x in 'xy']

        baseline = cm.sinh(x + 5 * y)
        gc_x = gm.GC(x)
        gc_y = gm.GC(5 * y)

        # Generate gradients
        gc_x[gm.DiffSymbol(x)]
        gc_y[gm.DiffSymbol(y)]

        gc_xy = gm.sinh(gc_x + gc_y)

        self.assertTrue(gm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(x)], gm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(y)], gm.diff(baseline, y), default_range=[-10, 10]))

    def test_cosh(self):
        x, y = [gm.Position(x) for x in 'xy']

        baseline = cm.cosh(x + 5 * y)
        gc_x = gm.GC(x)
        gc_y = gm.GC(5 * y)

        # Generate gradients
        gc_x[gm.DiffSymbol(x)]
        gc_y[gm.DiffSymbol(y)]

        gc_xy = gm.cosh(gc_x + gc_y)

        self.assertTrue(gm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(x)], gm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(y)], gm.diff(baseline, y), default_range=[-10, 10]))

    def test_tanh(self):
        x, y = [gm.Position(x) for x in 'xy']

        baseline = cm.tanh(x + 5 * y)
        gc_x = gm.GC(x)
        gc_y = gm.GC(5 * y)

        # Generate gradients
        gc_x[gm.DiffSymbol(x)]
        gc_y[gm.DiffSymbol(y)]

        gc_xy = gm.tanh(gc_x + gc_y)

        self.assertTrue(gm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(x)], gm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(y)], gm.diff(baseline, y), default_range=[-10, 10]))

    def test_asinh(self):
        x, y = [gm.Position(x) for x in 'xy']

        baseline = cm.asinh(x + 5 * y)
        gc_x = gm.GC(x)
        gc_y = gm.GC(5 * y)

        # Generate gradients
        gc_x[gm.DiffSymbol(x)]
        gc_y[gm.DiffSymbol(y)]

        gc_xy = gm.asinh(gc_x + gc_y)

        self.assertTrue(gm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(x)], gm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(y)], gm.diff(baseline, y), default_range=[-10, 10]))

    def test_acosh(self):
        x, y = [gm.Position(x) for x in 'xy']

        baseline = cm.acosh(x + 5 * y)
        gc_x = gm.GC(x)
        gc_y = gm.GC(5 * y)

        # Generate gradients
        gc_x[gm.DiffSymbol(x)]
        gc_y[gm.DiffSymbol(y)]

        gc_xy = gm.acosh(gc_x + gc_y)

        self.assertTrue(gm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(x)], gm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(y)], gm.diff(baseline, y), default_range=[-10, 10]))

    def test_atanh(self):
        x, y = [gm.Position(x) for x in 'xy']

        baseline = cm.atanh(x + 5 * y)
        gc_x = gm.GC(x)
        gc_y = gm.GC(5 * y)

        # Generate gradients
        gc_x[gm.DiffSymbol(x)]
        gc_y[gm.DiffSymbol(y)]

        gc_xy = gm.atanh(gc_x + gc_y)

        self.assertTrue(gm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(x)], gm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(y)], gm.diff(baseline, y), default_range=[-10, 10]))

    def test_exp(self):
        x, y = [gm.Position(x) for x in 'xy']

        baseline = cm.exp(x + 5 * y)
        gc_x = gm.GC(x)
        gc_y = gm.GC(5 * y)

        # Generate gradients
        gc_x[gm.DiffSymbol(x)]
        gc_y[gm.DiffSymbol(y)]

        gc_xy = gm.exp(gc_x + gc_y)

        self.assertTrue(gm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(x)], gm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(y)], gm.diff(baseline, y), default_range=[-10, 10]))

    def test_log(self):
        x, y = [gm.Position(x) for x in 'xy']

        baseline = cm.log(x + 5 * y)
        gc_x = gm.GC(x)
        gc_y = gm.GC(5 * y)

        # Generate gradients
        gc_x[gm.DiffSymbol(x)]
        gc_y[gm.DiffSymbol(y)]

        gc_xy = gm.log(gc_x + gc_y)

        self.assertTrue(gm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(x)], gm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(y)], gm.diff(baseline, y), default_range=[-10, 10]))

    def test_sqrt(self):
        x, y = [gm.Position(x) for x in 'xy']

        baseline = cm.sqrt(x + 5 * y)
        gc_x = gm.GC(x)
        gc_y = gm.GC(5 * y)

        # Generate gradients
        gc_x[gm.DiffSymbol(x)]
        gc_y[gm.DiffSymbol(y)]

        gc_xy = gm.sqrt(gc_x + gc_y)

        self.assertTrue(gm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(x)], gm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(y)], gm.diff(baseline, y), default_range=[-10, 10]))

    def test_abs(self):
        x, y = [gm.Position(x) for x in 'xy']

        baseline = gm.fake_abs(x + 5 * y)
        gc_x = gm.GC(x)
        gc_y = gm.GC(5 * y)

        # Generate gradients
        gc_x[gm.DiffSymbol(x)]
        gc_y[gm.DiffSymbol(y)]

        gc_xy = gm.abs(gc_x + gc_y)

        self.assertTrue(gm.numeric_eq(gc_xy.expr, baseline, default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(x)], gm.diff(baseline, x), default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gc_xy[gm.DiffSymbol(y)], gm.diff(baseline, y), default_range=[-10, 10]))


class TestMatrix(ut.TestCase):
    def test_operators(self):
        M = gm.Matrix([[1,2],[3,4],[5,6]])

        baseline = cm.dot(M, gm.Matrix([[4],[9]]))
        gm_M = gm.GM(M)
        gm_R = gm.dot(gm_M, gm.Matrix([[4],[9]]))

        self.assertTrue(gm.numeric_eq(gm_M.to_sym_matrix(), M, default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(gm_R.to_sym_matrix(), baseline, default_range=[-10, 10]))

    def test_transpose(self):
        M = gm.Matrix([[1,2],[3,4],[5,6]])

        baseline = M.T
        gm_M = gm.GM(M).T

        for e, x in zip(gm_M, gm.GM(baseline)):
            self.assertTrue(gm.numeric_eq(e.expr, x.expr, default_range=[-10, 10]))


if __name__ == '__main__':
    ut.main()
