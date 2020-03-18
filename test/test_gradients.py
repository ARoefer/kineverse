import unittest as ut

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

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[DiffSymbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[DiffSymbol(y)], baseline.diff(y))

    def test_subtraction(self):
        x, y = [Position(x) for x in 'xy']

        baseline = x - y
        gc_x = GC(x)
        gc_y = GC(y)
        
        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = gc_x - gc_y

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[DiffSymbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[DiffSymbol(y)], baseline.diff(y))


    def test_multiplication(self):
        x, y = [Position(x) for x in 'xy']

        baseline = x * (5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)
        
        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = gc_x * gc_y

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[DiffSymbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[DiffSymbol(y)], baseline.diff(y))


    def test_division(self):
        x, y = [Position(x) for x in 'xy']

        baseline = x / (5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)
        
        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = gc_x / gc_y

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[DiffSymbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[DiffSymbol(y)], baseline.diff(y))


    def testow(self):
        x, y = [Position(x) for x in 'xy']

        baseline = x ** (4 * y)
        gc_x = GC(x)
        gc_y = GC(4 * y)
        
        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = gc_x ** gc_y

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[DiffSymbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[DiffSymbol(y)], baseline.diff(y))


class TestFunctions(ut.TestCase):

    def test_sin(self):
        x, y = [Position(x) for x in 'xy']

        baseline = se.sin(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = sin(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[DiffSymbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[DiffSymbol(y)], baseline.diff(y))

    def test_cos(self):
        x, y = [Position(x) for x in 'xy']

        baseline = se.cos(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = cos(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[DiffSymbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[DiffSymbol(y)], baseline.diff(y))

    def test_tan(self):
        x, y = [Position(x) for x in 'xy']

        baseline = se.tan(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = tan(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[DiffSymbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[DiffSymbol(y)], baseline.diff(y))

    def test_asin(self):
        x, y = [Position(x) for x in 'xy']

        baseline = se.asin(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = asin(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[DiffSymbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[DiffSymbol(y)], baseline.diff(y))

    def test_acos(self):
        x, y = [Position(x) for x in 'xy']

        baseline = se.acos(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = acos(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[DiffSymbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[DiffSymbol(y)], baseline.diff(y))

    def test_atan(self):
        x, y = [Position(x) for x in 'xy']

        baseline = se.atan(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = atan(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[DiffSymbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[DiffSymbol(y)], baseline.diff(y))

    def test_sinh(self):
        x, y = [Position(x) for x in 'xy']

        baseline = se.sinh(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = sinh(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[DiffSymbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[DiffSymbol(y)], baseline.diff(y))

    def test_cosh(self):
        x, y = [Position(x) for x in 'xy']

        baseline = se.cosh(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = cosh(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[DiffSymbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[DiffSymbol(y)], baseline.diff(y))

    def test_tanh(self):
        x, y = [Position(x) for x in 'xy']

        baseline = se.tanh(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = tanh(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[DiffSymbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[DiffSymbol(y)], baseline.diff(y))

    def test_asinh(self):
        x, y = [Position(x) for x in 'xy']

        baseline = se.asinh(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = asinh(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[DiffSymbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[DiffSymbol(y)], baseline.diff(y))

    def test_acosh(self):
        x, y = [Position(x) for x in 'xy']

        baseline = se.acosh(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = acosh(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[DiffSymbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[DiffSymbol(y)], baseline.diff(y))

    def test_atanh(self):
        x, y = [Position(x) for x in 'xy']

        baseline = se.atanh(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = atanh(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[DiffSymbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[DiffSymbol(y)], baseline.diff(y))

    def test_exp(self):
        x, y = [Position(x) for x in 'xy']

        baseline = se.exp(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = exp(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[DiffSymbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[DiffSymbol(y)], baseline.diff(y))

    def test_log(self):
        x, y = [Position(x) for x in 'xy']

        baseline = se.log(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = log(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[DiffSymbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[DiffSymbol(y)], baseline.diff(y))

    def test_sqrt(self):
        x, y = [Position(x) for x in 'xy']

        baseline = se.sqrt(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = sqrt(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[DiffSymbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[DiffSymbol(y)], baseline.diff(y))

    def test_abs(self):
        x, y = [Position(x) for x in 'xy']

        baseline = abs(x + 5 * y)
        gc_x = GC(x)
        gc_y = GC(5 * y)

        # Generate gradients
        gc_x[DiffSymbol(x)]
        gc_y[DiffSymbol(y)]

        gc_xy = abs(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[DiffSymbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[DiffSymbol(y)], baseline.diff(y))


class TestMatrix(ut.TestCase):
    def test_operators(self):
        M = se.Matrix([[1,2],[3,4],[5,6]])

        baseline = M * se.Matrix([[4],[9]])
        gm_M = GM(M)
        gm_R = gm_M * se.Matrix([[4],[9]])

        self.assertEqual(gm_M, GM(M))
        self.assertEqual(gm_R, GM(baseline))

    def test_transpose(self):
        M = se.Matrix([[1,2],[3,4],[5,6]])

        baseline = M.T
        gm_M = GM(M).T

        for e, x in zip(gm_M, GM(baseline)):
            self.assertEqual(e, x)


if __name__ == '__main__':
    ut.main()
