SYM_MATH_ENGINE = 'CASADI'


if SYM_MATH_ENGINE == 'CASADI':
    import casadi as ca

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

    zeros = ca.SX.zeros 

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
            return set(ca.symvar(expression))
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

    def dot(*matrices):
        return ca.mtimes(matrices)

    def eye(n):
        return ca.SX.eye(n)

    def to_numpy(matrix):
        return np.array(matrix.tolist()).astype(float).reshape(matrix.shape)

    def diff(expression, symbol):
        return ca.jacobian(expression, Matrix(symbol))

    def eq_expr(a, b):
        # print(ca.simplify(a), ca.simplify(b))
        return ca.is_equal(ca.simplify(a), ca.simplify(b), 10000)

elif SYM_MATH_ENGINE == 'SYMENGINE':
    import symengine as se

    Symbol = se.Symbol
    Matrix = se.Matrix

    zeros  = se.zeros
    eye    = se.eye
    
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
        return (a.T*b)[0]

    def to_numpy(matrix):
        return np.array([float(x) for x in matrix]).astype(float).reshape(matrix.shape)

    def diff(expression, symbol):
        return expression.diff(symbol)

    def eq_expr(a, b):
        return a == b


def vector3(x, y, z):
    return Matrix([x, y, z, 0])

def point3(x, y, z):
    return Matrix([x, y, z, 1])


def position_of(frame):
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


def rotation_of(frame):
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
