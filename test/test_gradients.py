import unittest as ut

from kineverse.gradients.gradient_math import *


class TestOperators(ut.TestCase):

    def test_addition(self):
        x, y = spw.sp.symbols('x_p y_p')

        baseline = x + y
        gc_x = GradientContainer(x)
        gc_y = GradientContainer(y)
        
        # Generate gradients
        gc_x[get_diff_symbol(x)]
        gc_y[get_diff_symbol(y)]

        gc_xy = gc_x + gc_y

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[get_diff_symbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[get_diff_symbol(y)], baseline.diff(y))

    def test_subtraction(self):
        x, y = spw.sp.symbols('x_p y_p')

        baseline = x - y
        gc_x = GradientContainer(x)
        gc_y = GradientContainer(y)
        
        # Generate gradients
        gc_x[get_diff_symbol(x)]
        gc_y[get_diff_symbol(y)]

        gc_xy = gc_x - gc_y

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[get_diff_symbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[get_diff_symbol(y)], baseline.diff(y))


    def test_multiplication(self):
        x, y = spw.sp.symbols('x_p y_p')

        baseline = x * (5 * y)
        gc_x = GradientContainer(x)
        gc_y = GradientContainer(5 * y)
        
        # Generate gradients
        gc_x[get_diff_symbol(x)]
        gc_y[get_diff_symbol(y)]

        gc_xy = gc_x * gc_y

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[get_diff_symbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[get_diff_symbol(y)], baseline.diff(y))


    def test_division(self):
        x, y = spw.sp.symbols('x_p y_p')

        baseline = x / (5 * y)
        gc_x = GradientContainer(x)
        gc_y = GradientContainer(5 * y)
        
        # Generate gradients
        gc_x[get_diff_symbol(x)]
        gc_y[get_diff_symbol(y)]

        gc_xy = gc_x / gc_y

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[get_diff_symbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[get_diff_symbol(y)], baseline.diff(y))


    def test_pow(self):
        x, y = spw.sp.symbols('x_p y_p')

        baseline = x ** (4 * y)
        gc_x = GradientContainer(x)
        gc_y = GradientContainer(4 * y)
        
        # Generate gradients
        gc_x[get_diff_symbol(x)]
        gc_y[get_diff_symbol(y)]

        gc_xy = gc_x ** gc_y

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[get_diff_symbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[get_diff_symbol(y)], baseline.diff(y))


class TestFunctions(ut.TestCase):

    def test_sin(self):
        x, y = spw.sp.symbols('x_p y_p')

        baseline = spw.sin(x + 5 * y)
        gc_x = GradientContainer(x)
        gc_y = GradientContainer(5 * y)

        # Generate gradients
        gc_x[get_diff_symbol(x)]
        gc_y[get_diff_symbol(y)]

        gc_xy = sin(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[get_diff_symbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[get_diff_symbol(y)], baseline.diff(y))

    def test_cos(self):
        x, y = spw.sp.symbols('x_p y_p')

        baseline = spw.cos(x + 5 * y)
        gc_x = GradientContainer(x)
        gc_y = GradientContainer(5 * y)

        # Generate gradients
        gc_x[get_diff_symbol(x)]
        gc_y[get_diff_symbol(y)]

        gc_xy = cos(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[get_diff_symbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[get_diff_symbol(y)], baseline.diff(y))

    def test_tan(self):
        x, y = spw.sp.symbols('x_p y_p')

        baseline = spw.tan(x + 5 * y)
        gc_x = GradientContainer(x)
        gc_y = GradientContainer(5 * y)

        # Generate gradients
        gc_x[get_diff_symbol(x)]
        gc_y[get_diff_symbol(y)]

        gc_xy = tan(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[get_diff_symbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[get_diff_symbol(y)], baseline.diff(y))

    def test_asin(self):
        x, y = spw.sp.symbols('x_p y_p')

        baseline = spw.asin(x + 5 * y)
        gc_x = GradientContainer(x)
        gc_y = GradientContainer(5 * y)

        # Generate gradients
        gc_x[get_diff_symbol(x)]
        gc_y[get_diff_symbol(y)]

        gc_xy = asin(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[get_diff_symbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[get_diff_symbol(y)], baseline.diff(y))

    def test_acos(self):
        x, y = spw.sp.symbols('x_p y_p')

        baseline = spw.acos(x + 5 * y)
        gc_x = GradientContainer(x)
        gc_y = GradientContainer(5 * y)

        # Generate gradients
        gc_x[get_diff_symbol(x)]
        gc_y[get_diff_symbol(y)]

        gc_xy = acos(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[get_diff_symbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[get_diff_symbol(y)], baseline.diff(y))

    def test_atan(self):
        x, y = spw.sp.symbols('x_p y_p')

        baseline = spw.atan(x + 5 * y)
        gc_x = GradientContainer(x)
        gc_y = GradientContainer(5 * y)

        # Generate gradients
        gc_x[get_diff_symbol(x)]
        gc_y[get_diff_symbol(y)]

        gc_xy = atan(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[get_diff_symbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[get_diff_symbol(y)], baseline.diff(y))

    def test_sinh(self):
        x, y = spw.sp.symbols('x_p y_p')

        baseline = spw.sp.sinh(x + 5 * y)
        gc_x = GradientContainer(x)
        gc_y = GradientContainer(5 * y)

        # Generate gradients
        gc_x[get_diff_symbol(x)]
        gc_y[get_diff_symbol(y)]

        gc_xy = sinh(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[get_diff_symbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[get_diff_symbol(y)], baseline.diff(y))

    def test_cosh(self):
        x, y = spw.sp.symbols('x_p y_p')

        baseline = spw.sp.cosh(x + 5 * y)
        gc_x = GradientContainer(x)
        gc_y = GradientContainer(5 * y)

        # Generate gradients
        gc_x[get_diff_symbol(x)]
        gc_y[get_diff_symbol(y)]

        gc_xy = cosh(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[get_diff_symbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[get_diff_symbol(y)], baseline.diff(y))

    def test_tanh(self):
        x, y = spw.sp.symbols('x_p y_p')

        baseline = spw.sp.tanh(x + 5 * y)
        gc_x = GradientContainer(x)
        gc_y = GradientContainer(5 * y)

        # Generate gradients
        gc_x[get_diff_symbol(x)]
        gc_y[get_diff_symbol(y)]

        gc_xy = tanh(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[get_diff_symbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[get_diff_symbol(y)], baseline.diff(y))

    def test_asinh(self):
        x, y = spw.sp.symbols('x_p y_p')

        baseline = spw.sp.asinh(x + 5 * y)
        gc_x = GradientContainer(x)
        gc_y = GradientContainer(5 * y)

        # Generate gradients
        gc_x[get_diff_symbol(x)]
        gc_y[get_diff_symbol(y)]

        gc_xy = asinh(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[get_diff_symbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[get_diff_symbol(y)], baseline.diff(y))

    def test_acosh(self):
        x, y = spw.sp.symbols('x_p y_p')

        baseline = spw.sp.acosh(x + 5 * y)
        gc_x = GradientContainer(x)
        gc_y = GradientContainer(5 * y)

        # Generate gradients
        gc_x[get_diff_symbol(x)]
        gc_y[get_diff_symbol(y)]

        gc_xy = acosh(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[get_diff_symbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[get_diff_symbol(y)], baseline.diff(y))

    def test_atanh(self):
        x, y = spw.sp.symbols('x_p y_p')

        baseline = spw.sp.atanh(x + 5 * y)
        gc_x = GradientContainer(x)
        gc_y = GradientContainer(5 * y)

        # Generate gradients
        gc_x[get_diff_symbol(x)]
        gc_y[get_diff_symbol(y)]

        gc_xy = atanh(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[get_diff_symbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[get_diff_symbol(y)], baseline.diff(y))

    def test_exp(self):
        x, y = spw.sp.symbols('x_p y_p')

        baseline = spw.sp.exp(x + 5 * y)
        gc_x = GradientContainer(x)
        gc_y = GradientContainer(5 * y)

        # Generate gradients
        gc_x[get_diff_symbol(x)]
        gc_y[get_diff_symbol(y)]

        gc_xy = exp(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[get_diff_symbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[get_diff_symbol(y)], baseline.diff(y))

    def test_log(self):
        x, y = spw.sp.symbols('x_p y_p')

        baseline = spw.sp.log(x + 5 * y)
        gc_x = GradientContainer(x)
        gc_y = GradientContainer(5 * y)

        # Generate gradients
        gc_x[get_diff_symbol(x)]
        gc_y[get_diff_symbol(y)]

        gc_xy = log(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[get_diff_symbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[get_diff_symbol(y)], baseline.diff(y))

    def test_sqrt(self):
        x, y = spw.sp.symbols('x_p y_p')

        baseline = spw.sqrt(x + 5 * y)
        gc_x = GradientContainer(x)
        gc_y = GradientContainer(5 * y)

        # Generate gradients
        gc_x[get_diff_symbol(x)]
        gc_y[get_diff_symbol(y)]

        gc_xy = sqrt(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[get_diff_symbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[get_diff_symbol(y)], baseline.diff(y))

    def test_abs(self):
        x, y = spw.sp.symbols('x_p y_p')

        baseline = spw.fake_Abs(x + 5 * y)
        gc_x = GradientContainer(x)
        gc_y = GradientContainer(5 * y)

        # Generate gradients
        gc_x[get_diff_symbol(x)]
        gc_y[get_diff_symbol(y)]

        gc_xy = abs(gc_x + gc_y)

        self.assertEqual(gc_xy.expr, baseline)
        self.assertEqual(gc_xy[get_diff_symbol(x)], baseline.diff(x))
        self.assertEqual(gc_xy[get_diff_symbol(y)], baseline.diff(y))


class TestMatrix(ut.TestCase):
    def test_operators(self):
        M = spw.sp.Matrix([[1,2],[3,4],[5,6]])

        baseline = M * spw.sp.Matrix([[4],[9]])
        gm_M = GradientMatrix(M)
        gm_R = gm_M * spw.sp.Matrix([[4],[9]])

        self.assertEqual(gm_M.expr, M)
        self.assertEqual(gm_R.expr, baseline)

    def test_transpose(self):
        M = spw.sp.Matrix([[1,2],[3,4],[5,6]])

        baseline = M.T
        gm_M = GradientMatrix(M).T

        for e, x in zip(gm_M, baseline):
            self.assertEqual(e.expr, x)


if __name__ == '__main__':
    ut.main()