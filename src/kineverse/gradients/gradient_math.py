"""
The gradient_math module implements common mathematical functions which are compatible
with augemented gradients.
Additionally, it provides constructors for 3d transformations.
"""


import kineverse.gradients.common_math as cm

from kineverse.gradients.diff_logic         import Position, \
                                                   Velocity, \
                                                   Acceleration, \
                                                   Jerk, \
                                                   Snap, \
                                                   Symbol, \
                                                   DiffSymbol, \
                                                   IntSymbol
from kineverse.gradients.gradient_container import GradientContainer as GC, \
                                                   GradientMatrix    as GM 
from kineverse.symengine_types              import symengine_types, symengine_matrix_types

contrast = 1e10


if cm.SYM_MATH_ENGINE == 'CASADI':
    """In this section, the infix operators for casadi are overridden
    to provide compatibility with augmented gradients. Unlike symengine's
    operators, casadi seems to default to __getitem__ when it is lhs in an
    infix operation.
    """

    def op_wrapper(cls, name_o_op):
        o_op = getattr(cls, '__{}__'.format(name_o_op))

        def wrapped_op(self, other):
            if type(other) in {GC, GM}:
                return getattr(other, '__r{}__'.format(name_o_op))(self)
            return o_op(self, other)

        return wrapped_op
    

    cm.ca.SX.__add__ = op_wrapper(cm.ca.SX, 'add')
    cm.ca.MX.__add__ = op_wrapper(cm.ca.MX, 'add')
    cm.ca.DM.__add__ = op_wrapper(cm.ca.DM, 'add')

    cm.ca.SX.__sub__ = op_wrapper(cm.ca.SX, 'sub')
    cm.ca.MX.__sub__ = op_wrapper(cm.ca.MX, 'sub')
    cm.ca.DM.__sub__ = op_wrapper(cm.ca.DM, 'sub')

    cm.ca.SX.__mul__ = op_wrapper(cm.ca.SX, 'mul')
    cm.ca.MX.__mul__ = op_wrapper(cm.ca.MX, 'mul')
    cm.ca.DM.__mul__ = op_wrapper(cm.ca.DM, 'mul')

    cm.ca.SX.__div__ = op_wrapper(cm.ca.SX, 'div')
    cm.ca.MX.__div__ = op_wrapper(cm.ca.MX, 'div')
    cm.ca.DM.__div__ = op_wrapper(cm.ca.DM, 'div')

is_symbol = cm.is_symbol
eq_expr   = cm.eq_expr
eye       = cm.eye
pos_of    = cm.pos_of
rot_of    = cm.rot_of
x_of      = cm.x_of
y_of      = cm.y_of
z_of      = cm.z_of
free_symbols = cm.free_symbols 
diff      = cm.diff
Matrix    = cm.Matrix
trace     = cm.trace


def is_symbolic(expr):
    """Is the given expression symbolic?"""
    return (type(expr) == GC and len(expr.free_symbols) > 0) or cm.is_symbolic(expr)

def is_matrix(expr):
    """Is the given expression a matrix?"""
    return type(expr) == GM or cm.is_matrix(expr)

def extract_expr(expr):
    """Extract the expression, if given object is an augmented gradient."""
    return expr if type(expr) != GC else expr.expr

def wrap_expr(expr):
    """Wrap the expression in an augmented gradient, if it is not one already."""
    return expr if type(expr) == GC else GC(expr)

def get_diff(term, symbols=None):
    """Returns the derivative of a passed expression."""
    if cm.is_symbol(term):
        return DiffSymbol(term)
    
    if type(term) != GC:
        term = GC(term)

    if symbols is None:
        term.do_full_diff()
        return sum([s * t for s, t in term.gradients.items()])
    else:
        return sum([s * term[s] for s in symbols if s in term])

subs = cm.subs

def sin(expr):
    """Sine"""
    if type(expr) == GC:
        return GC(cm.sin(expr.expr), {s: cm.cos(expr.expr) * d for s, d in expr.gradients.items()})
    return cm.sin(expr)

def cos(expr):
    """Cosine"""
    if type(expr) == GC:
        return GC(cm.cos(expr.expr), {s: -cm.sin(expr.expr) * d for s, d in expr.gradients.items()})
    return cm.cos(expr)

def tan(expr):
    """Tangent"""
    if type(expr) == GC:
        return GC(cm.tan(expr.expr), {s: d * (1 + cm.tan(expr.expr)**2) for s, d in expr.gradients.items()})
    return cm.tan(expr)

def asin(expr):
    """Arcsine"""
    if type(expr) == GC:
        return GC(cm.asin(expr.expr), {s: d / cm.sqrt(1 - expr.expr**2) for s, d in expr.gradients.items()})
    return cm.asin(expr)

def acos(expr):
    """Arccosine"""
    if type(expr) == GC:
        return GC(cm.acos(expr.expr), {s: -d / cm.sqrt(1 - expr.expr**2) for s, d in expr.gradients.items()})
    return cm.acos(expr)

def atan(expr):
    """Arctangent"""
    if type(expr) == GC:
        return GC(cm.atan(expr.expr), {s: d / (1 + expr.expr**2) for s, d in expr.gradients.items()})
    return cm.atan(expr)

def sinh(expr):
    """Hyperbolic sine"""
    if type(expr) == GC:
        return GC(cm.sinh(expr.expr), {s: d * cm.cosh(expr.expr) for s, d in expr.gradients.items()})
    return cm.sinh(expr)

def cosh(expr):
    """Hyperbolic cosine"""
    if type(expr) == GC:
        return GC(cm.cosh(expr.expr), {s: d * cm.sinh(expr.expr) for s, d in expr.gradients.items()})
    return cm.cosh(expr)

def tanh(expr):
    """Hyperbolic tangent"""
    if type(expr) == GC:
        return GC(cm.tanh(expr.expr), {s: d * (1 - cm.tanh(expr.expr)**2) for s, d in expr.gradients.items()})
    return cm.tanh(expr)

def asinh(expr):
    """Hyperbolic arcsine"""
    if type(expr) == GC:
        return GC(cm.asinh(expr.expr), {s: d / cm.sqrt(expr.expr**2 + 1) for s, d in expr.gradients.items()})
    return cm.asinh(expr)

def acosh(expr):
    """Hyperbolic arccosine"""
    if type(expr) == GC:
        return GC(cm.acosh(expr.expr), {s: d / cm.sqrt(expr.expr**2 - 1) for s, d in expr.gradients.items()})
    return cm.acosh(expr)

def atanh(expr):
    """Hyperbolic arctangent"""
    if type(expr) == GC:
        return GC(cm.atanh(expr.expr), {s: d / (1 - expr.expr**2) for s, d in expr.gradients.items()})
    return cm.atanh(expr)


def exp(expr):
    """Exponential"""
    if type(expr) == GC:
        return GC(cm.exp(expr.expr), {s: d * cm.exp(expr.expr) for s, d in expr.gradients.items()})
    return cm.exp(expr)
    
def log(expr):
    """Logarithm"""
    if type(expr) == GC:
        return GC(cm.log(expr.expr), {s: d / expr.expr for s, d in expr.gradients.items()})
    return cm.log(expr)

def sqrt(expr):
    """Square root"""
    if type(expr) == GC:
        return GC(cm.sqrt(expr.expr), {s: d / (2 * cm.sqrt(expr.expr)) for s, d in expr.gradients.items()})
    return cm.sqrt(expr)

def fake_abs(expr):
    return sqrt(expr**2)

def abs(expr):
    """Absolute value"""
    if type(expr) == GC:
        return GC(fake_abs(expr.expr), {s: d * expr.expr / cm.sqrt(expr.expr ** 2) for s, d in expr.gradients.items()})
    return fake_abs(expr)

def is_gradient(m_list):
    """Returns a nested list of booleans indicating which of the original elements are gradient containers."""
    return max([type(x) == GC if type(x) != list and type(x) != tuple else is_gradient(x) for x in m_list])

def matrix_wrapper(m_list):
    """Converts a nested input list to a GradientMatrix or Matrix, depending on whether GradientContainers are in the input."""
    if is_gradient(m_list):
        return GM(m_list)
    return cm.Matrix(m_list)


def point3(x, y, z):
    """Creates a 3d point for homogeneous transformations."""
    a  = [x, y, z]
    mf = GM if max([type(v) == GC for v in a]) else cm.Matrix
    return mf([x, y, z, 1])

def vector3(x, y, z):
    """Creates a 3d vector for homogeneous transformations."""
    a  = [x, y, z]
    mf = GM if max([type(v) == GC for v in a]) else cm.Matrix
    return mf([x, y, z, 0])

unitX = vector3(1, 0, 0)
unitY = vector3(0, 1, 0)
unitZ = vector3(0, 0, 1)

def norm(v):
    if type(v) in cm.matrix_types:
        return cm.norm(v)
    
    out = 0
    for x in v:
        out += x ** 2
    return sqrt(out)

dot  = cm.dot
dot_product = cm.dot_product

def cross(u, v):
    """Computes the cross product between two vectors."""
    return matrix_wrapper([u[1] * v[2] - u[2] * v[1],
                           u[2] * v[0] - u[0] * v[2],
                           u[0] * v[1] - u[1] * v[0], 0])

def axis_angle_from_matrix(rotation_matrix):
    rm = rotation_matrix
    if hasattr(rm, 'free_symbols') and len(rm.free_symbols) == 0:
        if 1 - rm[0,0] <= 1e-4 and 1 - rm[1,1] <= 1e-4 and 1 - rm[2,2] <= 1e-4:
            return unitX, 0
    
    angle = (trace(rm[:3, :3]) - 1) / 2
    angle = acos(angle)
    angle = angle if type(angle) is not cm.ComplexDouble else angle.real
    x = (rm[2, 1] - rm[1, 2])
    y = (rm[0, 2] - rm[2, 0])
    z = (rm[1, 0] - rm[0, 1])
    n = sqrt(x * x + y * y + z * z)


    axis = vector3(x / n, y / n, z / n)
    return axis, angle

def translation3(x, y, z, w=1):
    """Creates a homogenous translation transformation."""
    a  = [x, y, z, w]
    mf = GM if max([type(v) == GC for v in a]) else cm.Matrix
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
    mf = GM if max([type(v) == GC for v in a]) else cm.Matrix

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
    return dot(rz, ry, rx)


def rotation3_axis_angle(axis, angle):
    """ Conversion of unit axis and angle to 4x4 rotation matrix according to:
        http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/index.htm
    """
    mf = GM if type(angle) == GC or type(axis) == GM else cm.Matrix
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
    mf = GM if max([type(v) == GC for v in a]) else cm.Matrix
    x2 = x * x
    y2 = y * y
    z2 = z * z
    w2 = w * w
    return mf([[w2 + x2 - y2 - z2, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y, 0],
               [2 * x * y + 2 * w * z, w2 - x2 + y2 - z2, 2 * y * z - 2 * w * x, 0],
               [2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, w2 - x2 - y2 + z2, 0],
               [0, 0, 0, 1]])


def frame3_axis_angle(axis, angle, loc):
    return dot(translation3(loc[0], loc[1], loc[2]), rotation3_axis_angle(axis, angle))


def frame3_rpy(r, p, y, loc):
    return dot(translation3(loc[0], loc[1], loc[2]), rotation3_rpy(r, p, y))


def frame3_quaternion(x, y, z, qx, qy, qz, qw):
    return dot(translation3(x, y, z), rotation3_quaternion(qx, qy, qz, qw))


def inverse_frame(frame):
    mf = GM if type(frame) == GM else cm.Matrix
    inv = mf(cm.eye(4))
    inv[:3, :3] = frame[:3, :3].T
    inv[:3, 3] = dot(-inv[:3, :3], frame[:3, 3])
    return inv

def merge_gradients_add(ga, gb):
    """Helper function that merges two gradients by adding them,
    without adding the main expressions."""
    out = ga.copy()
    for s, g in gb.items():
        if s in out:
            out[s] += g
        else:
            out[s]  = g
    return out


def greater_than(x, y):
    """Creates a gradient approximating the :math:`x > y` expression. The gradient contains a fake derivative mapping the velocity of x to True and the velocity of y to False."""
    # if cm.SYM_MATH_ENGINE == 'SYMENGINE':
    fake_diffs = {}
    if cm.is_symbol(y):
        fake_diffs[DiffSymbol(y)] = -1
    else:
        if type(y) in cm.math_types: 
            y = GC(y)
        if type(y) == GC:
            y.do_full_diff()
            fake_diffs = {s: -g for s, g in y.gradients.items()}
    if cm.is_symbol(x):
        x_d = DiffSymbol(x)
        if x_d in fake_diffs:
            fake_diffs[x_d] += 1
        else:
            fake_diffs[x_d] = 1
    else:
        if type(x) in cm.math_types: 
            x = GC(x)
        if type(x) == GC:
            x.do_full_diff()
            fake_diffs = merge_gradients_add(fake_diffs, x.gradients)
    return GC(0.5 * tanh((x - y) * contrast) + 0.5, fake_diffs)

    # if type(x) not in cm.math_types and type(y) not in cm.math_types:
    #     return cm.ca.SX(x) > y
    # return x > y


def less_than(x, y):
    """Creates a gradient approximating the :math:`x < y` expression. The gradient contains a fake derivative mapping the velocity of x to False and the velocity of y to True."""
    return greater_than(y, x)

def alg_and(x, y):
    """Creates a gradient approximating the :math:`x \wedge y` expression by means of multiplication. x, y are assumed to be boolean approximations resulting in 1 for truth and 0 for falsehood."""
    if cm.SYM_MATH_ENGINE == 'SYMENGINE':
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
