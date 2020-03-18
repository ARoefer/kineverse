import symengine as se

from kineverse.gradients.diff_logic         import get_diff_symbol, \
                                                   create_symbol, \
                                                   create_pos, \
                                                   create_vel, \
                                                   create_acc, \
                                                   create_jerk, \
                                                   create_snap, \
                                                   Position, \
                                                   Velocity, \
                                                   Acceleration, \
                                                   Jerk, \
                                                   Snap, \
                                                   Symbol, \
                                                   DiffSymbol
from kineverse.gradients.gradient_container import GradientContainer as GC
from kineverse.gradients.gradient_matrix    import GradientMatrix    as GM
from kineverse.symengine_types              import symengine_types, symengine_matrix_types

contrast = 1e10


def extract_expr(expr):
    return expr if type(expr) != GC else expr.expr

def wrap_expr(expr):
    return expr if type(expr) == GC else GC(expr)

def get_diff(term, symbols=None):
    """Returns the derivative of a passed expression."""
    if type(term) == Symbol:
        return get_diff_symbol(term)
    
    if type(term) != GC:
        term = GC(term)

    if symbols is None:
        term.do_full_diff()
        return sum([s * t for s, t in term.gradients.items()])
    else:
        return sum([s * term[s] for s in symbols if s in term])

def subs(expr, subs_dict):
    if hasattr(expr, 'subs') and callable(expr.subs):
        return expr.subs(subs_dict)
    return expr

def sin(expr):
    """Sine"""
    if type(expr) == GC:
        return GC(se.sin(expr.expr), {s: se.cos(expr.expr) * d for s, d in expr.gradients.items()})
    return se.sin(expr)

def cos(expr):
    """Cosine"""
    if type(expr) == GC:
        return GC(se.cos(expr.expr), {s: -se.sin(expr.expr) * d for s, d in expr.gradients.items()})
    return se.cos(expr)

def tan(expr):
    """Tangent"""
    if type(expr) == GC:
        return GC(se.tan(expr.expr), {s: d * (1 + se.tan(expr.expr)**2) for s, d in expr.gradients.items()})
    return se.tan(expr)

def asin(expr):
    """Arcsine"""
    if type(expr) == GC:
        return GC(se.asin(expr.expr), {s: d / se.sqrt(1 - expr.expr**2) for s, d in expr.gradients.items()})
    return se.asin(expr)

def acos(expr):
    """Arccosine"""
    if type(expr) == GC:
        return GC(se.acos(expr.expr), {s: -d / se.sqrt(1 - expr.expr**2) for s, d in expr.gradients.items()})
    return se.acos(expr)

def atan(expr):
    """Arctangent"""
    if type(expr) == GC:
        return GC(se.atan(expr.expr), {s: d / (1 + expr.expr**2) for s, d in expr.gradients.items()})
    return se.atan(expr)

def sinh(expr):
    """Hyperbolic sine"""
    if type(expr) == GC:
        return GC(se.sinh(expr.expr), {s: d * se.cosh(expr.expr) for s, d in expr.gradients.items()})
    return se.sinh(expr)

def cosh(expr):
    """Hyperbolic cosine"""
    if type(expr) == GC:
        return GC(se.cosh(expr.expr), {s: d * se.sinh(expr.expr) for s, d in expr.gradients.items()})
    return se.cosh(expr)

def tanh(expr):
    """Hyperbolic tangent"""
    if type(expr) == GC:
        return GC(se.tanh(expr.expr), {s: d * (1 - se.tanh(expr.expr)**2) for s, d in expr.gradients.items()})
    return se.tanh(expr)

def asinh(expr):
    """Hyperbolic arcsine"""
    if type(expr) == GC:
        return GC(se.asinh(expr.expr), {s: d / se.sqrt(expr.expr**2 + 1) for s, d in expr.gradients.items()})
    return se.asinh(expr)

def acosh(expr):
    """Hyperbolic arccosine"""
    if type(expr) == GC:
        return GC(se.acosh(expr.expr), {s: d / se.sqrt(expr.expr**2 - 1) for s, d in expr.gradients.items()})
    return se.acosh(expr)

def atanh(expr):
    """Hyperbolic arctangent"""
    if type(expr) == GC:
        return GC(se.atanh(expr.expr), {s: d / (1 - expr.expr**2) for s, d in expr.gradients.items()})
    return se.atanh(expr)


def exp(expr):
    """Exponential"""
    if type(expr) == GC:
        return GC(se.exp(expr.expr), {s: d * se.exp(expr.expr) for s, d in expr.gradients.items()})
    return se.exp(expr)
    
def log(expr):
    """Logarithm"""
    if type(expr) == GC:
        return GC(se.log(expr.expr), {s: d / expr.expr for s, d in expr.gradients.items()})
    return se.log(expr)

def sqrt(expr):
    """Square root"""
    if type(expr) == GC:
        return GC(se.sqrt(expr.expr), {s: d / (2 * se.sqrt(expr.expr)) for s, d in expr.gradients.items()})
    return se.sqrt(expr)

def fake_abs(expr):
    return sqrt(expr**2)

def abs(expr):
    """Absolute value"""
    if type(expr) == GC:
        return GC(fake_abs(expr.expr), {s: d * expr.expr / se.sqrt(expr.expr ** 2) for s, d in expr.gradients.items()})
    return fake_abs(expr)

def is_gradient(m_list):
    """Returns a nested list of booleans indicating which of the original elements are gradient containers."""
    return max([type(x) == GC if type(x) != list and type(x) != tuple else is_gradient(x) for x in m_list])

def matrix_wrapper(m_list):
    """Converts a nested input list to a GradientMatrix or smyengine.Matrix, depending on whether GradientContainers are in the input."""
    if is_gradient(m_list):
        return GM(m_list)
    return se.Matrix(m_list)


def point3(x, y, z):
    """Creates a 3d point for homogenous transformations."""
    a  = [x, y, z]
    mf = GM if max([type(v) == GC for v in a]) else se.Matrix
    return mf([x, y, z, 1])

def vector3(x, y, z):
    """Creates a 3d vector for homogenous transformations."""
    a  = [x, y, z]
    mf = GM if max([type(v) == GC for v in a]) else se.Matrix
    return mf([x, y, z, 0])

unitX = vector3(1, 0, 0)
unitY = vector3(0, 1, 0)
unitZ = vector3(0, 0, 1)

def norm(v):
    """Computes the L2 norm (sqrt of sum of squares) of the input iterable."""
    r = 0
    for x in v:
        r += x ** 2
    return sqrt(r)

def cross(u, v):
    """Computes the cross product between two vecotrs."""
    return matrix_wrapper([u[1] * v[2] - u[2] * v[1],
                           u[2] * v[0] - u[0] * v[2],
                           u[0] * v[1] - u[1] * v[0], 0])

def dot(a, b):
    return (a.T*b)[0]

def pos_of(frame):
    return frame[:4, 3:]

def trans_of(frame):
    return se.eye(3).col_join(se.Matrix([[0] * 3])).row_join(frame[:4, 3:])

def rot_of(frame):
    return frame[:4, :3].row_join(se.Matrix([0, 0, 0, 1]))

def x_of(frame):
    return frame[:4, :1]

def y_of(frame):
    return frame[:4, 1:2]

def z_of(frame):
    return frame[:4, 2:3]

def trace(matrix):
    return sum(matrix[i, i] for i in range(matrix.shape[0]))

def axis_angle_from_matrix(rotation_matrix):
    rm = rotation_matrix
    if hasattr(rm, 'free_symbols') and len(rm.free_symbols) == 0:
        if 1 - rm[0,0] <= 1e-4 and 1 - rm[1,1] <= 1e-4 and 1 - rm[2,2] <= 1e-4:
            return unitX, 0
    
    angle = (trace(rm[:3, :3]) - 1) / 2
    angle = acos(angle)
    angle = angle if type(angle) is not se.ComplexDouble else angle.real
    x = (rm[2, 1] - rm[1, 2])
    y = (rm[0, 2] - rm[2, 0])
    z = (rm[1, 0] - rm[0, 1])
    n = sqrt(x * x + y * y + z * z)


    axis = vector3(x / n, y / n, z / n)
    return axis, angle

def translation3(x, y, z, w=1):
    """Creates a homogenous translation transformation."""
    a  = [x, y, z, w]
    mf = GM if max([type(v) == GC for v in a]) else se.Matrix
    return mf([[1, 0, 0, x],
               [0, 1, 0, y],
               [0, 0, 1, z],
               [0, 0, 0, w]])


def rotation3_rpy(roll, pitch, yaw):
    """ Conversion of roll, pitch, yaw to 4x4 rotation matrix according to:
        https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/src/frames.cpp#L167
    """
    # TODO don't split this into 3 matrices

    a  = [roll, pitch, yaw]
    mf = GM if max([type(v) == GC for v in a]) else se.Matrix

    rx = mf([[1, 0, 0, 0],
             [0, cos(roll), -sin(roll), 0],
             [0, sin(roll), cos(roll), 0],
             [0, 0, 0, 1]])
    ry = mf([[cos(pitch), 0, sin(pitch), 0],
             [0, 1, 0, 0],
             [-sin(pitch), 0, cos(pitch), 0],
             [0, 0, 0, 1]])
    rz = mf([[cos(yaw), -sin(yaw), 0, 0],
             [sin(yaw), cos(yaw), 0, 0],
             [0, 0, 1, 0],
             [0, 0, 0, 1]])
    return (rz * ry * rx)


def rotation3_axis_angle(axis, angle):
    """ Conversion of unit axis and angle to 4x4 rotation matrix according to:
        http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/index.htm
    """
    mf = GM if type(angle) == GC or type(axis) == GM else se.Matrix
    ct = cos(angle)
    st = sin(angle)
    vt = 1 - ct
    m_vt_0 = vt * axis[0]
    m_vt_1 = vt * axis[1]
    m_vt_2 = vt * axis[2]
    m_st_0 = axis[0] * st
    m_st_1 = axis[1] * st
    m_st_2 = axis[2] * st
    m_vt_0_1 = m_vt_0 * axis[1]
    m_vt_0_2 = m_vt_0 * axis[2]
    m_vt_1_2 = m_vt_1 * axis[2]
    return mf([[ct + m_vt_0 * axis[0], -m_st_2 + m_vt_0_1, m_st_1 + m_vt_0_2, 0],
               [m_st_2 + m_vt_0_1, ct + m_vt_1 * axis[1], -m_st_0 + m_vt_1_2, 0],
               [-m_st_1 + m_vt_0_2, m_st_0 + m_vt_1_2, ct + m_vt_2 * axis[2], 0],
               [0, 0, 0, 1]])


def rotation3_quaternion(x, y, z, w):
    """ Unit quaternion to 4x4 rotation matrix according to:
        https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/src/frames.cpp#L167
    """
    a  = [x, y, z, w]
    mf = GM if max([type(v) == GC for v in a]) else se.Matrix
    x2 = x * x
    y2 = y * y
    z2 = z * z
    w2 = w * w
    return mf([[w2 + x2 - y2 - z2, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y, 0],
               [2 * x * y + 2 * w * z, w2 - x2 + y2 - z2, 2 * y * z - 2 * w * x, 0],
               [2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, w2 - x2 - y2 + z2, 0],
               [0, 0, 0, 1]])


def frame3_axis_angle(axis, angle, loc):
    return translation3(*loc) * rotation3_axis_angle(axis, angle)


def frame3_rpy(r, p, y, loc):
    return translation3(*loc) * rotation3_rpy(r, p, y)


def frame3_quaternion(x, y, z, qx, qy, qz, qw):
    return translation3(x, y, z) * rotation3_quaternion(qx, qy, qz, qw)


def inverse_frame(frame):
    mf = GM if type(frame) == GM else se.Matrix
    inv = mf(se.eye(4))
    inv[:3, :3] = frame[:3, :3].T
    inv[:3, 3] = -inv[:3, :3] * frame[:3, 3]
    return inv

def wrap_matrix_mul(a, b):
    if type(a) == GC or type(b) == GC:
        if type(a) in symengine_matrix_types:
            return GM(a) * b
        elif type(b) in symengine_matrix_types:
            return GM(b) * a
    return a * b


def merge_gradients_add(ga, gb):
    out = ga.copy()
    for s, g in gb.items():
        if s in out:
            out[s] += g
        else:
            out[s]  = g
    return out


def greater_than(x, y):
    """Creates a gradient approximating the :math:`x > y` expression. The gradient contains a fake derivative mapping the velocity of x to True and the velocity of y to False."""
    fake_diffs = {}
    if type(y) == Symbol:
        fake_diffs[get_diff_symbol(y)] = -1
    else:
        if type(y) in symengine_types: 
            y = GC(y)
        if type(y) == GC:
            y.do_full_diff()
            fake_diffs = {s: -g for s, g in y.gradients.items()}
    if type(x) == Symbol:
        x_d = get_diff_symbol(x)
        if x_d in fake_diffs:
            fake_diffs[x_d] += 1
        else:
            fake_diffs[x_d] = 1
    else:
        if type(x) in symengine_types: 
            x = GC(x)
        if type(x) == GC:
            x.do_full_diff()
            fake_diffs = merge_gradients_add(fake_diffs, x.gradients)
    return GC(0.5 * tanh((x - y) * contrast) + 0.5, fake_diffs)

def less_than(x, y):
    """Creates a gradient approximating the :math:`x < y` expression. The gradient contains a fake derivative mapping the velocity of x to False and the velocity of y to Ture."""
    return greater_than(y, x)

def alg_and(x, y):
    """Creates a gradient approximating the :math:`x \wedge y` expression by means of multiplication. x, y are assumed to be boolean approximations resulting in 1 for truth and 0 for falsehood."""
    if type(x) is GC:
        if type(y) in symengine_types:
            y = GC(y)

        if type(y) is GC:
            y.do_full_diff()
            return GC(x.expr * y.expr, merge_gradients_add(x.gradients, y.gradients))
        return GC(x.expr * y, x.gradients)
    elif type(y) is GC:
        if type(x) in symengine_types:
            x = GC(x)
        if type(x) is GC:
            x.do_full_diff()
            return GC(y.expr * y.expr, merge_gradients_add(x.gradients, y.gradients))
        return GC(y.expr * x, y.gradients)

    return x * y

def alg_not(x):
    """inverts the truth value of boolean approximation by subtracting it from 1."""
    return 1 - x

def alg_or(x, y):
    """Creates a gradient approximating the :math:`x \vee y` expression by means of multiplication. x, y are assumed to be boolean approximations resulting in 1 for truth and 0 for falsehood."""
    return alg_not(alg_and(alg_not(x), alg_not(y)))
