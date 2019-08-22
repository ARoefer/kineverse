import os
import symengine as sp
import numpy     as np

import yaml
import copy      as copy_module
import importlib

from collections                  import namedtuple
from giskardpy.symengine_wrappers import *
from kineverse.time_wrapper       import Time
from kineverse.type_sets          import symengine_types
from sensor_msgs.msg              import JointState as JointStateMsg
from iai_bullet_sim.utils         import Frame, Vector3, Point3
from visualization_msgs.msg       import Marker as MarkerMsg

pi = 3.14159265359
rad2deg = 57.2957795131
deg2rad = 1.0 / rad2deg

def symbol_formatter(symbol_name):
    if '__' in symbol_name:
        raise Exception('Illegal character sequence in symbol name "{}"! Double underscore "__" is a separator sequence.'.format(symbol_name))
    return sp.Symbol(symbol_name.replace('/', '__'))

def fake_heaviside(expr):
    return 0 if expr <= 0 else 1

def res_pkg_path(rpath):
    """Resolves a ROS package relative path to a global path.
    :param rpath: Potential ROS URI to resolve.
    :type rpath: str
    :return: Local file system path
    :rtype: str
    """
    if rpath[:10] == 'package://':
        paths = os.environ['ROS_PACKAGE_PATH'].split(':')

        rpath = rpath[10:]
        pkg = rpath[:rpath.find('/')]

        for rpp in paths:
            if rpp[rpp.rfind('/') + 1:] == pkg:
                return '{}/{}'.format(rpp[:rpp.rfind('/')], rpath)
            if os.path.isdir('{}/{}'.format(rpp, pkg)):
                return '{}/{}'.format(rpp, rpath)
        raise Exception('Package "{}" can not be found in ROS_PACKAGE_PATH!'.format(pkg))
    return rpath

def make_pkg_path(path):
    paths = os.environ['ROS_PACKAGE_PATH'].split(':')
    for p in paths:
        if path[:len(p)] == p:
            return 'package://{}/{}'.format(p[p.rfind('/') + 1:], path[len(p) + 1:])
    return path

def import_class(class_path):
    """Imports a class using a type string.

    :param class_path: Type string of the class.
    :type  class_path: str
    :rtype: type
    """
    components = class_path.split('.')
    mod = importlib.import_module('.'.join(components[:-1]))
    try:
        return getattr(mod, components[-1])
    except AttributeError:
        raise AttributeError('Module "{}" loaded from "{}" has no attribute "{}"'.format(mod.__name__, '.'.join(components[:-1]), components[-1]))
        


def saturate(x, low=0, high=1):
    breadth_scale = 6 / high - low
    return 1 / (1 + sp.exp(-2* ( x * breadth_scale + low - 3)))


def tip_at_one(x):
    return -4*x**2 + 8 * x - 3


def abs(x):
    return sp.sqrt(x**2)


def cmdDictToJointState(command):
    js = JointStateMsg()
    js.header.stamp = Time.now()
    for joint, vel in command.items():
        js.name.append(str(joint))
        js.position.append(0)
        js.velocity.append(vel)
        js.effort.append(0)

    return js

def subs_if_sym(var, subs_dict):
    t = type(var)
    if t == int or t == float or t == str:
        return var
    else:
        return var.subs(subs_dict)

# Returns [x,y,z,w]
QuatT = namedtuple('QuatT', ['x', 'y', 'z', 'w'])
RPY = namedtuple('RPY', ['r', 'p', 'y'])

def rot3_to_quat(rot3):
    w  = sp.sqrt(1 + rot3[0,0] + rot3[1,1] + rot3[2,2]) * 0.5
    w4 = 4 * w
    x  = (rot3[2,1] - rot3[1,2]) / w4
    y  = (rot3[0,2] - rot3[2,0]) / w4
    z  = (rot3[1,0] - rot3[0,1]) / w4
    return QuatT(x,y,z,w)


def rot3_to_rpy(rot3, evaluate=False):
    sy = sp.sqrt(rot3[0,0] * rot3[0,0] + rot3[2,2] * rot3[2,2])

    if sy >= 1e-6:
        if evaluate:
            return RPY(sp.atan2(rot3[2,1], rot3[2,2]).evalf(real=True), sp.atan2(-rot3[2,0], sy).evalf(real=True), sp.atan2(rot3[1,0], rot3[0,0]).evalf(real=True))
        else:
            return RPY(sp.atan2(rot3[2,1], rot3[2,2]), sp.atan2(-rot3[2,0], sy), sp.atan2(rot3[1,0], rot3[0,0]))
    else:
        if evaluate:
            return RPY(sp.atan2(-rot3[1,2], rot3[1,1]).evalf(real=True), sp.atan2(-rot3[2,0], sy).evalf(real=True), 0)
        else:
            return RPY(sp.atan2(-rot3[1,2], rot3[1,1]), sp.atan2(-rot3[2,0], sy), 0)


def real_quat_from_matrix(frame):
    tr = frame[0,0] + frame[1,1] + frame[2,2]

    if tr > 0:
        S = sqrt(tr+1.0) * 2 # S=4*qw
        qw = 0.25 * S
        qx = (frame[2,1] - frame[1,2]) / S
        qy = (frame[0,2] - frame[2,0]) / S
        qz = (frame[1,0] - frame[0,1]) / S
    elif frame[0,0] > frame[1,1] and frame[0,0] > frame[2,2]:
        S  = sqrt(1.0 + frame[0,0] - frame[1,1] - frame[2,2]) * 2 # S=4*qx
        qw = (frame[2,1] - frame[1,2]) / S
        qx = 0.25 * S
        qy = (frame[0,1] + frame[1,0]) / S
        qz = (frame[0,2] + frame[2,0]) / S
    elif frame[1,1] > frame[2,2]:
        S  = sqrt(1.0 + frame[1,1] - frame[0,0] - frame[2,2]) * 2 # S=4*qy
        qw = (frame[0,2] - frame[2,0]) / S
        qx = (frame[0,1] + frame[1,0]) / S
        qy = 0.25 * S
        qz = (frame[1,2] + frame[2,1]) / S
    else:
        S  = sqrt(1.0 + frame[2,2] - frame[0,0] - frame[1,1]) * 2 # S=4*qz
        qw = (frame[1,0] - frame[0,1]) / S
        qx = (frame[0,2] + frame[2,0]) / S
        qy = (frame[1,2] + frame[2,1]) / S
        qz = 0.25 * S
    return (qx, qy, qz, qw)


def jsDictToJSMsg(js_dict):
    js = JointStateMsg()
    js.header.stamp = Time.now()
    for joint, state in js_dict.items():
        js.name.append(joint)
        js.position.append(state.position)
        js.velocity.append(state.velocity)
        js.effort.append(state.effort)

    return js

def copy(obj):
    if type(obj) in symengine_types:
        return obj
    return copy_module.copy(obj)

def deepcopy(obj):
    if type(obj) in symengine_types:
        return obj
    out = copy_module.deepcopy(obj)
    if out is None and obj is not None:
        raise Exception('Deep copy of {} failed! Please implement a custom __deepcopy__ function for type {}.'.format(obj, type(obj)))
    return out

class Blank:
    def __str__(self):
        return '\n'.join(['{}: {}'.format(field, str(getattr(self, field))) for field in dir(self) if field[0] != '_' and not callable(getattr(self, field))])

    def __deepcopy__(self, memo):
        out = Blank()
        for attrn in [x  for x in dir(self) if x[0] != '_']:
            attr = getattr(self, attrn)
            if type(attr) in symengine_types:
                setattr(out, attrn, attr)
            else:
                setattr(out, attrn, copy_module.deepcopy(attr, memo))
        memo[id(self)] = out
        return out


def bb(**kwargs):
    out = Blank()
    for k, v in kwargs.items():
        setattr(out, k, v)
    return out


def visualize_obj(obj, display, pose, ns='objects', color=None):
    if DLShape.is_a(obj):
        if color is None and hasattr(obj, 'color'):
            color = obj.color


        if DLCube.is_a(obj):
            display.draw_cube(ns, pose, (obj.length, obj.width, obj.height), color[0], color[1], color[2], color[3])
        elif DLCylinder.is_a(obj):
            display.draw_cylinder(ns, pose, obj.height, obj.radius, color[0], color[1], color[2], color[3])
        elif DLSphere.is_a(obj):
            display.draw_sphere(ns, pos_of(pose), obj.radius, color[0], color[1], color[2], color[3])

    if DLCompoundObject.is_a(obj):
        if type(obj.subObject) == list:
            for x in obj.subObject:
                visualize_obj(x, display, pose * x.pose, ns, color)
        else:
            visualize_obj(obj.subObject, display, pose * obj.subObject.pose, ns, color)
