SYM_MATH_ENGINE = 'CASADI'


if SYM_MATH_ENGINE == 'CASADI':
    import casadi as ca
    import numpy  as np

    print('New instance of __SYMBOL_CACHE created')

    __SYMBOL_CACHE = {}

    def Symbol(data):
        if isinstance(data, str) or isinstance(data, unicode):
            if data not in __SYMBOL_CACHE:
                __SYMBOL_CACHE[data] = ca.SX.sym(data)
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
                    m[i] = data[i]
            return m

    ca.SX.__mul__ = ca.mtimes
    ca.MX.__mul__ = ca.mtimes
    ca.DM.__mul__ = ca.mtimes

    zeros  = ca.SX.zeros
    dot    = ca.dot

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

    def free_symbols(expression):
        if type(expression) == ca.SX or type(expression) == ca.MX:
            return {Symbol(str(x)) for x in ca.symvar(expression)}
        if hasattr(expression, 'free_symbols'):
            return expression.free_symbols
        return set()

    def subs(expr, subs_dict):
        if hasattr(expr, 'subs') and callable(expr.subs):
            return expr.subs(subs_dict)
        return expr

    def cross(a, b):
        return ca.cross(a, b)

    def norm(v):
        """Computes the L2 norm (sqrt of sum of squares) of the input iterable."""
        return ca.norm_2(v)

    def diag(*args):
        return ca.diag(args)

    def eye(n):
        return ca.SX.eye(n)

    def to_numpy(matrix):
        return np.array(matrix.elements()).astype(float).reshape(matrix.shape)

    def diff(expression, symbol):
        return ca.jacobian(expression, Matrix(symbol))

    def eq_expr(a, b):
        # print(ca.simplify(a), ca.simplify(b))
        return ca.is_equal(ca.simplify(a), ca.simplify(b), 10000)

    def is_symbol(expr):
        return expr.shape[0] * expr.shape[1] == 1

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

        width = sum([m.shape[0] for m in matrices])
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
            filtered_args = [kwargs[k] for k in self.str_params]
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
        print(str_params)
        try:
            f = ca.Function('f', [Matrix(params)], [ca.densify(function)])
        except:
            f = ca.Function('f', [Matrix(params)], ca.densify(function))
        return CompiledFunction(str_params, f, 0, function.shape)


elif SYM_MATH_ENGINE == 'SYMENGINE':
    import symengine as se

    symengine_matrix_types = set([se.DenseMatrix, 
                                  se.ImmutableMatrix, 
                                  se.ImmutableDenseMatrix, 
                                  se.MatrixBase, 
                                  se.MutableDenseMatrix])

    symengine_types  = set([getattr(se.lib.symengine_wrapper, x) for x in dir(se.lib.symengine_wrapper) if type(getattr(se.lib.symengine_wrapper, x)) == type])
    symengine_floats = {se.RealDouble, se.RealNumber}

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


    def free_symbols(expression):
        return expression.free_symbols

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

    def dot(a, b):
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
        return type(expr) in symengine_types and len(expr.free_symbols) > 0

    def is_matrix(expr):
        return type(expr) in symengine_matrix_types

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
    frame[0,3] = 0
    frame[1,3] = 0
    frame[2,3] = 0
    return frame


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
