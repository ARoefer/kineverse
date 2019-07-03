import giskardpy.symengine_wrappers as spw

from kineverse.gradients.gradient_container import GradientContainer as GC
from kineverse.gradients.gradient_matrix    import GradientMatrix    as GM

def sin(expr):
    if type(expr) == GC:
        return GC(spw.sin(expr.expr), {s: spw.cos(expr.expr) * d for s, d in expr.gradients.items()})
    return spw.sin(expr)

def cos(expr):
    if type(expr) == GC:
        return GC(spw.cos(expr.expr), {s: -spw.sin(expr.expr) * d for s, d in expr.gradients.items()})
    return spw.cos(expr)

def tan(expr):
    if type(expr) == GC:
        return GC(spw.tan(expr.expr), {s: d * (1 + spw.tan(expr.expr)**2) for s, d in expr.gradients.items()})
    return spw.tan(expr)

def asin(expr):
    if type(expr) == GC:
        return GC(spw.asin(expr.expr), {s: d / spw.sqrt(1 - expr.expr**2) for s, d in expr.gradients.items()})
    return spw.asin(expr)

def acos(expr):
    if type(expr) == GC:
        return GC(spw.acos(expr.expr), {s: -d / spw.sqrt(1 - expr.expr**2) for s, d in expr.gradients.items()})
    return spw.acos(expr)

def atan(expr):
    if type(expr) == GC:
        return GC(spw.atan(expr.expr), {s: d / (1 + expr.expr**2) for s, d in expr.gradients.items()})
    return spw.atan(expr)

def sinh(expr):
    if type(expr) == GC:
        return GC(spw.sp.sinh(expr.expr), {s: d * spw.sp.cosh(expr.expr) for s, d in expr.gradients.items()})
    return spw.sp.sinh(expr)

def cosh(expr):
    if type(expr) == GC:
        return GC(spw.sp.cosh(expr.expr), {s: d * spw.sp.sinh(expr.expr) for s, d in expr.gradients.items()})
    return spw.sp.cosh(expr)

def tanh(expr):
    if type(expr) == GC:
        return GC(spw.sp.tanh(expr.expr), {s: d * (1 - spw.sp.tanh(expr.expr)**2) for s, d in expr.gradients.items()})
    return spw.sp.tanh(expr)

def asinh(expr):
    if type(expr) == GC:
        return GC(spw.sp.asinh(expr.expr), {s: d / spw.sqrt(expr.expr**2 + 1) for s, d in expr.gradients.items()})
    return spw.sp.asinh(expr)

def acosh(expr):
    if type(expr) == GC:
        return GC(spw.sp.acosh(expr.expr), {s: d / spw.sqrt(expr.expr**2 - 1) for s, d in expr.gradients.items()})
    return spw.sp.acosh(expr)

def atanh(expr):
    if type(expr) == GC:
        return GC(spw.sp.atanh(expr.expr), {s: d / (1 - expr.expr**2) for s, d in expr.gradients.items()})
    return spw.sp.atanh(expr)


def exp(expr):
    if type(expr) == GC:
        return GC(spw.sp.exp(expr.expr), {s: d * spw.sp.exp(expr.expr) for s, d in expr.gradients.items()})
    return spw.exp(expr)
    
def log(expr):
    if type(expr) == GC:
        return GC(spw.log(expr.expr), {s: d / expr.expr for s, d in expr.gradients.items()})
    return spw.log(expr)

def sqrt(expr):
    if type(expr) == GC:
        return GC(spw.sqrt(expr.expr), {s: d / (2 * spw.sqrt(expr.expr)) for s, d in expr.gradients.items()})
    return spw.sqrt(expr)

def abs(expr):
    if type(expr) == GC:
        return GC(spw.fake_Abs(expr.expr), {s: d * expr.expr / spw.sqrt(expr.expr ** 2) for s, d in expr.gradients.items()})
    return spw.fake_Abs(expr)

def norm(v):
    r = 0
    for x in v:
        r += x ** 2
    return sqrt(r)

def cross(u, v):
    if type(u) == GM or type(v) == GM:
        return GM([u[1] * v[2] - u[2] * v[1],
                   u[2] * v[0] - u[0] * v[2],
                   u[0] * v[1] - u[1] * v[0], 0])
    return spw.sp.Matrix([u[1] * v[2] - u[2] * v[1],
                      u[2] * v[0] - u[0] * v[2],
                      u[0] * v[1] - u[1] * v[0], 0])

def translation3(x, y, z, w=1):
    a = [x, y, z, w]
    if max([type(v) == GC for v in a]):
        return GM(spw.translation3(*[v.expr if type(v) == GC else v for v in a]), 
                  [([None] * 3) + [x] if type(x) == GC else [None]* 4 for x in a])
    return spw.translation3(x, y, z, w)


def rotation3_rpy(roll, pitch, yaw):
    """ Conversion of roll, pitch, yaw to 4x4 rotation matrix according to:
        https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/src/frames.cpp#L167
    """
    # TODO don't split this into 3 matrices

    a = [roll, pitch, yaw]
    if max([type(v) == GC for v in a]):
        rx = GM([[1, 0, 0, 0],
                 [0, cos(roll), -sin(roll), 0],
                 [0, sin(roll), cos(roll), 0],
                 [0, 0, 0, 1]])
        ry = GM([[cos(pitch), 0, sin(pitch), 0],
                 [0, 1, 0, 0],
                 [-sin(pitch), 0, cos(pitch), 0],
                 [0, 0, 0, 1]])
        rz = GM([[cos(yaw), -sin(yaw), 0, 0],
                 [sin(yaw), cos(yaw), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])
        return (rz * ry * rx)
    return spw.rotation3_rpy(roll, pitch, yaw)


def rotation3_axis_angle(axis, angle):
    """ Conversion of unit axis and angle to 4x4 rotation matrix according to:
        http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/index.htm
    """
    if type(axis) == GM or type(angle) == GC:
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
        return GM([[ct + m_vt_0 * axis[0], -m_st_2 + m_vt_0_1, m_st_1 + m_vt_0_2, 0],
                      [m_st_2 + m_vt_0_1, ct + m_vt_1 * axis[1], -m_st_0 + m_vt_1_2, 0],
                      [-m_st_1 + m_vt_0_2, m_st_0 + m_vt_1_2, ct + m_vt_2 * axis[2], 0],
                      [0, 0, 0, 1]])
    return spw.rotation3_axis_angle(axis, angle)


def rotation3_quaternion(x, y, z, w):
    """ Unit quaternion to 4x4 rotation matrix according to:
        https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/src/frames.cpp#L167
    """
    a = [x, y, z, w]
    if max([type(v) == GC for v in a]):
        x2 = x * x
        y2 = y * y
        z2 = z * z
        w2 = w * w
        return GM([[w2 + x2 - y2 - z2, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y, 0],
                   [2 * x * y + 2 * w * z, w2 - x2 + y2 - z2, 2 * y * z - 2 * w * x, 0],
                   [2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, w2 - x2 - y2 + z2, 0],
                   [0, 0, 0, 1]])
    return spw.rotation3_quaternion(x, y, z, w)


def frame3_axis_angle(axis, angle, loc):
    return translation3(*loc) * rotation3_axis_angle(axis, angle)


def frame3_rpy(r, p, y, loc):
    return translation3(*loc) * rotation3_rpy(r, p, y)


def frame3_quaternion(x, y, z, qx, qy, qz, qw):
    return translation3(x, y, z) * rotation3_quaternion(qx, qy, qz, qw)


def inverse_frame(frame):
    if type(frame) == GM:
        inv = GM(spw.sp.eye(4))
        inv[:3, :3] = frame[:3, :3].T
        inv[:3, 3] = -inv[:3, :3] * frame[:3, 3]
        return inv
    return spw.inverse_frame(frame)