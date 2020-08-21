"""
The common_math module abstracts away from the specific symbolic engine used by Kineverse.
Currently, Symengine and Casadi are supported. The engine can be selected using
the SYM_MATH_ENGINE variable.

The module also contains some functions related to homogeneous transformations,
which should probably be moved.
"""

SYM_MATH_ENGINE = 'CASADI'
# SYM_MATH_ENGINE = 'SYMENGINE'

import numpy  as np


if SYM_MATH_ENGINE == 'CASADI':
    import casadi as ca

    print('New instance of __SYMBOL_CACHE created')

    __SYMBOL_CACHE = {}

    __osxsym = ca.SX.sym

    def symbol_trace(self, data):
        return __osxsym(data)

    ca.SX.sym = symbol_trace

    def Symbol(data):
        if isinstance(data, str) or isinstance(data, unicode):
            if data not in __SYMBOL_CACHE:
                __SYMBOL_CACHE[data] = ca.SX.sym(ca.SX(), data)
            return __SYMBOL_CACHE[data]
        raise Exception('Symbols can only be created from strings or unicode. Got: {}'.format(data))


    def Matrix(data):
        try:
            return ca.SX(data)
        except NotImplementedError:
            if hasattr(data, u'shape'):
                m = ca.SX(*data.shape)
            else:
                x = len(data)
                if isinstance(data[0], list) or isinstance(data[0], tuple):
                    y = len(data[0])
                else:
                    y = 1
                m = ca.SX(x, y)
            for i in range(m.shape[0]):
                if y > 1:
                    for j in range(m.shape[1]):
                        try:
                            m[i, j] = data[i][j]
                        except:
                            m[i, j] = data[i, j]
                else:
                    if isinstance(data[i], list) or isinstance(data[i], tuple):
                        m[i] = data[i][0]
                    else:
                        m[i] = data[i]
            return m

    def tracer(a, b):
        if type(a) in matrix_types and type(b) in matrix_types and is_matrix(a) and is_matrix(b):
            raise Exception('Tracer exception')
        return ca.mtimes(a, b)

    ca.SX.__mul__ = tracer
    ca.MX.__mul__ = tracer
    ca.DM.__mul__ = tracer

    zeros  = ca.SX.zeros

    acos   = ca.acos
    acosh  = ca.acosh
    asin   = ca.asin
    asinh  = ca.asinh
    atan   = ca.atan
    atan2  = ca.atan2
    atanh  = ca.atanh
    cos    = ca.cos
    cosh   = ca.cosh
    exp    = ca.exp
    log    = ca.log
    sin    = ca.sin
    sinh   = ca.sinh
    sqrt   = ca.sqrt
    tan    = ca.tan
    tanh   = ca.tanh

    eq     = ca.eq
    le     = ca.le
    ge     = ca.ge
    lt     = ca.lt
    gt     = ca.gt

    acos   = ca.acos
    acosh  = ca.acosh
    asin   = ca.asin
    asinh  = ca.asinh
    atan   = ca.atan
    atanh  = ca.atanh
    cos    = ca.cos
    cosh   = ca.cosh
    exp    = ca.exp
    log    = ca.log
    sin    = ca.sin
    sinh   = ca.sinh
    sqrt   = ca.sqrt
    tan    = ca.tan
    tanh   = ca.tanh

    eq     = ca.eq
    le     = ca.le
    ge     = ca.ge
    lt     = ca.lt
    gt     = ca.gt

    matrix_types = {ca.SX, ca.DM, ca.MX}
    math_types   = set(matrix_types)
    symfloats    = set()

    _CASADI_SUBS_CACHE = {}

    def dot_product(a, b):
        if type(a) in matrix_types and type(b) in matrix_types:
            return ca.dot(a, b)
        else:
            a = a.elements() if type(a) in matrix_types else a
            b = b.elements() if type(b) in matrix_types else b

            return sum([x * y for x, y in zip(a, b)])

    def norm(v):
        """Computes the L2 norm (sqrt of sum of squares) of the input iterable."""
        # return ca.norm_2(v)
        r = 0
        if type(v) in matrix_types:
            for x in v.elements():
                r += x ** 2
            return ca.sqrt(r)
        raise Exception('This function is only meant to compute norms of casadi matrices. This is of type {}'.format(type(v)))

    def free_symbols(expression):
        if type(expression) in math_types:
            if type(expression) is ca.DM:
                print(expression.shape, expression)
            return {Symbol(str(x)) for x in ca.symvar(expression)}
        if hasattr(expression, 'free_symbols'):
            return expression.free_symbols
        return set()

    def subs(expr, subs_dict):
        if hasattr(expr, 'subs') and callable(expr.subs):
            return expr.subs(subs_dict)
        if SYM_MATH_ENGINE == 'CASADI' and type(expr) in math_types:
            # if id(expr) not in _CASADI_SUBS_CACHE:
            #     fn = speed_up(expr, free_symbols(expr))
            #     _CASADI_SUBS_CACHE[id(expr)] = fn
            # return _CASADI_SUBS_CACHE[id(expr)](**{str(s): v for s, v in subs_dict.items()})
            return speed_up(expr, free_symbols(expr))(**{str(s): v for s, v in subs_dict.items()})
        return expr

    def cross(a, b):
        return ca.cross(a, b)

    def dot(*args):
        out = args[0]
        for a in args[1:]:
            if type(out) in matrix_types and type(a) in matrix_types:
                out = ca.mtimes(out, a)
            elif type(out) in matrix_types and type(a) not in matrix_types:
                out = a.rdot(out)
            else:
                out = out.dot(a)
        return out

    def diag(*args):
        return ca.diag(args)

    def eye(n):
        return ca.SX.eye(n)

    def to_list(matrix):
        if type(matrix) in matrix_types:
            return np.array(matrix.elements()).reshape((matrix.shape[1], matrix.shape[0])).T.tolist()
        return matrix.tolist()

    def to_numpy(matrix):
        if type(matrix) != np.ndarray:
            return np.array(matrix.elements()).astype(float).reshape((matrix.shape[1], matrix.shape[0])).T
        return matrix

    def diff(expression, symbol):
        return ca.jacobian(expression, Matrix(symbol))

    def eq_expr(a, b):
        # print(type(a), type(b))
        if type(a) in math_types and type(b) in math_types:
            return ca.is_equal(ca.simplify(a), ca.simplify(b), 10000)
        return a == b

    def is_symbol(expr):
        if type(expr) in math_types:
            return expr.shape[0] * expr.shape[1] == 1 and expr.is_leaf() and len(free_symbols(expr)) > 0
        return False

    def is_matrix(expr):
        return hasattr(expr, 'shape') and expr.shape[0] * expr.shape[1] > 1

    def is_symbolic(expr):
        return len(free_symbols(expr)) > 0

    def vstack(*matrices):
        width = matrices[0].shape[1]
        for x, m in enumerate(matrices[1:]):
            if width != m.shape[1]:
                raise Exception('Matrices for stacking need to be of equal width. Initial width is {} but matrix {} has width {}'.format(width, x, m.shape[1]))

        height = sum([m.shape[0] for m in matrices])
        out    = ca.SX(height, width)
        
        start_row = 0
        for m in matrices:
            for y in range(m.shape[0]):
                for x in range(m.shape[1]):
                    out[start_row + y, x] = m[y, x]
            start_row += m.shape[0]

        return out

    def hstack(*matrices):
        height = matrices[0].shape[0]
        for x, m in enumerate(matrices[1:]):
            if height != m.shape[0]:
                raise Exception('Matrices for stacking need to be of equal height. Initial height is {} but matrix {} has height {}'.format(height, x, m.shape[0]))

        width = sum([m.shape[1] for m in matrices])
        out   = ca.SX(height, width)
        
        start_col = 0
        for m in matrices:
            for x in range(m.shape[1]):
                for y in range(m.shape[0]):
                    out[y, start_col + x] = m[y, x]
            start_col += m.shape[1]

        return out

    class CompiledFunction(object):
        def __init__(self, str_params, fast_f, l, shape):
            self.str_params = str_params
            self.fast_f = fast_f
            self.shape = shape
            self.buf, self.f_eval = fast_f.buffer()
            self.out = np.zeros(self.shape, order='F')
            self.buf.set_res(0, memoryview(self.out))

        def __call__(self, **kwargs):
            filtered_args = [float(kwargs[k]) for k in self.str_params]
            return self.call2(filtered_args)

        def call2(self, filtered_args):
            """
            :param filtered_args: parameter values in the same order as in self.str_params
            :type filtered_args: list
            :return:
            """

            filtered_args = np.array(filtered_args, dtype=float)
            self.buf.set_arg(0, memoryview(filtered_args))
            self.f_eval()
            return self.out

    def speed_up(function, parameters, backend=u'clang'):
        params     = list(parameters)
        str_params = [str(x) for x in params]
        # print(str_params)
        try:
            f = ca.Function('f', [Matrix(params)], [ca.densify(function)])
        except:
            f = ca.Function('f', [Matrix(params)], ca.densify(function))
        return CompiledFunction(str_params, f, 0, function.shape)

    def numeric_eq(a, b, default_range=[-1, 1], samples=1000, precision=1e-4, verbose=0):
        vars_a = free_symbols(a)
        vars_b = free_symbols(b)

        if vars_a != vars_b:
            return False

        eval_a = speed_up(a, vars_a)
        eval_b = speed_up(b, vars_b)

        params = np.random.rand(samples, len(vars_a)) * (default_range[1] - default_range[0]) + default_range[0]
        for x in params:
            v_a = eval_a.call2(x)
            v_b = eval_b.call2(x)

            delta = np.max(np.abs(v_a - v_b))
            if delta > precision:
                if verbose > 0:
                    print('Delta greater than {}. Delta is {}. Results:\na = \n{}\nb = \n{}'.format(precision, delta, v_a, v_b))
                return False

        return True


elif SYM_MATH_ENGINE == 'SYMENGINE':
    import symengine as se

    matrix_types = {se.DenseMatrix,
                    se.ImmutableMatrix,
                    se.ImmutableDenseMatrix,
                    se.MatrixBase,
                    se.MutableDenseMatrix}

    math_types   = set([getattr(se.lib.symengine_wrapper, x) for x in dir(se.lib.symengine_wrapper) if type(getattr(se.lib.symengine_wrapper, x)) == type])
    symfloats = {se.RealDouble, se.RealNumber}

    Symbol = se.Symbol
    Matrix = se.Matrix

    zeros  = se.zeros
    eye    = se.eye

    diag   = se.diag
    
    acos   = se.acos
    acosh  = se.acosh
    asin   = se.asin
    asinh  = se.asinh
    atan   = se.atan
    atanh  = se.atanh
    cos    = se.cos
    cosh   = se.cosh
    exp    = se.exp
    log    = se.log
    sin    = se.sin
    sinh   = se.sinh
    sqrt   = se.sqrt
    tan    = se.tan
    tanh   = se.tanh

    import math
    atan2 = math.atan2


    def free_symbols(expression):
        if hasattr(expression, 'free_symbols'):
            return expression.free_symbols
        return set()

    def subs(expr, subs_dict):
        if hasattr(expr, 'subs') and callable(expr.subs):
            return expr.subs(subs_dict)
        return expr

    def norm(v):
        """Computes the L2 norm (sqrt of sum of squares) of the input iterable."""
        r = 0
        for x in v:
            r += x ** 2
        return se.sqrt(r)

    def dot(*args):
        out = args[0]
        for a in args:
            out *= a
        return out

    def dot_product(a, b):
        return (a.T * b)[0]

    def to_numpy(matrix):
        return np.array([float(x) for x in matrix]).astype(float).reshape(matrix.shape)

    def diff(expression, symbol):
        return expression.diff(symbol)

    def eq_expr(a, b):
        return a == b

    def is_symbol(expr):
        return type(expr) == se.Symbol

    def is_symbolic(expr):
        return type(expr) in math_types and len(expr.free_symbols) > 0

    def is_matrix(expr):
        return type(expr) in matrix_types

    def vstack(*matrices):
        out = matrices[0]
        for m in matrices[1:]:
            out = out.col_join(m)
        return out

    def hstack(*matrices):
        out = matrices[0]
        for m in matrices[1:]:
            out = out.row_join(m)
        return out

    from kineverse.gradients.llvm_wrapper import speed_up



def vector3(x, y, z):
    return Matrix([x, y, z, 0])

def point3(x, y, z):
    return Matrix([x, y, z, 1])


def pos_of(frame):
    """
    :param frame: 4x4 Matrix
    :type frame: Matrix
    :return: 4x1 Matrix; the translation part of a frame in form of a point
    :rtype: Matrix
    """
    return frame[:4, 3:]


def translation_of(frame):
    """
    :param frame: 4x4 Matrix
    :type frame: Matrix
    :return: 4x4 Matrix; sets the rotation part of a frame to identity
    :rtype: Matrix
    """
    r = eye(4)
    r[0, 3] = frame[0, 3]
    r[1, 3] = frame[1, 3]
    r[2, 3] = frame[2, 3]
    return r


def rot_of(frame):
    """
    :param frame: 4x4 Matrix
    :type frame: Matrix
    :return: 4x4 Matrix; sets the translation part of a frame to 0
    :rtype: Matrix
    """
    out = frame[:, :]
    out[0,3] = 0
    out[1,3] = 0
    out[2,3] = 0
    return out


def trace(matrix):
    """
    :type matrix: Matrix
    :rtype: Union[float, Symbol]
    """
    return sum(matrix[i, i] for i in range(matrix.shape[0]))


def x_of(frame):
    return frame[:4, :1]


def y_of(frame):
    return frame[:4, 1:2]


def z_of(frame):
    return frame[:4, 2:3]


def inverse_frame(frame):
    """
    :param frame: 4x4 Matrix
    :type frame: Matrix
    :return: 4x4 Matrix
    :rtype: Matrix
    """
    inv = eye(4)
    inv[:3, :3] = frame[:3, :3].T
    inv[:3, 3] = dot(-inv[:3, :3], frame[:3, 3])
    return inv
