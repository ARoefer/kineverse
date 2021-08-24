import os

import copy  as copy_module
import importlib
import math
import numpy as np
import kineverse.gradients.gradient_math as gm

from collections                       import namedtuple
from kineverse.time_wrapper            import Time

if gm.cm.SYM_MATH_ENGINE == 'SYMENGINE':
    from kineverse.symengine_types         import symengine_types
else:
    symengine_types = set()


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
    parts = path.split('/')[1:]
    for x in reversed(range(len(parts) + 1)):
        if os.path.isfile('/{}/package.xml'.format('/'.join(parts[:x]))):
            return 'package://{}'.format('/'.join(parts[x-1:]))
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

# Returns [x,y,z,w]
RPY = namedtuple('RPY', ['r', 'p', 'y'])

def rot3_to_rpy(rot3, evaluate=False):
    sy = gm.sqrt(rot3[0,0] * rot3[0,0] + rot3[2,2] * rot3[2,2])

    if sy >= 1e-6:
        if evaluate:
            return RPY(float(gm.atan2(rot3[2,1], rot3[2,2])), 
                       float(gm.atan2(-rot3[2,0], sy)), 
                       float(gm.atan2(rot3[1,0], rot3[0,0])))
        else:
            return RPY(gm.atan2(rot3[2,1], rot3[2,2]), 
                       gm.atan2(-rot3[2,0], sy), 
                       gm.atan2(rot3[1,0], rot3[0,0]))
    else:
        if evaluate:
            return RPY(float(gm.atan2(-rot3[1,2], rot3[1,1])), 
                       float(gm.atan2(-rot3[2,0], sy)), 0)
        else:
            return RPY(gm.atan2(-rot3[1,2], rot3[1,1]), 
                       gm.atan2(-rot3[2,0], sy), 0)


def real_quat_from_matrix(frame):
    tr = frame[0,0] + frame[1,1] + frame[2,2]

    if tr > 0:
        S = math.sqrt(tr+1.0) * 2 # S=4*qw
        qw = 0.25 * S
        qx = (frame[2,1] - frame[1,2]) / S
        qy = (frame[0,2] - frame[2,0]) / S
        qz = (frame[1,0] - frame[0,1]) / S
    elif frame[0,0] > frame[1,1] and frame[0,0] > frame[2,2]:
        S  = math.sqrt(1.0 + frame[0,0] - frame[1,1] - frame[2,2]) * 2 # S=4*qx
        qw = (frame[2,1] - frame[1,2]) / S
        qx = 0.25 * S
        qy = (frame[0,1] + frame[1,0]) / S
        qz = (frame[0,2] + frame[2,0]) / S
    elif frame[1,1] > frame[2,2]:
        S  = math.sqrt(1.0 + frame[1,1] - frame[0,0] - frame[2,2]) * 2 # S=4*qy
        qw = (frame[0,2] - frame[2,0]) / S
        qx = (frame[0,1] + frame[1,0]) / S
        qy = 0.25 * S
        qz = (frame[1,2] + frame[2,1]) / S
    else:
        S  = math.sqrt(1.0 + frame[2,2] - frame[0,0] - frame[1,1]) * 2 # S=4*qz
        qw = (frame[1,0] - frame[0,1]) / S
        qx = (frame[0,2] + frame[2,0]) / S
        qy = (frame[1,2] + frame[2,1]) / S
        qz = 0.25 * S
    return (float(qx), float(qy), float(qz), float(qw))


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


def union(sets, starting_set=None):
    out = set() if starting_set is None else starting_set
    for s in sets:
        out = out.union(s)
    return out


def static_var_bounds(km, symbols):
    """Extracts the static bounds for a set of symbols from a model
       and returns them as a numpy matrix.
    
    Args:
        km (ArticulationModel): Articulation model to draw info from
        symbols (set): Set of symbols to bound
    
    Returns:
        (list, np.ndarray, list): Returns a tuple of m bounded variables,
                                  a bounding m*2 bound matrix, and n unbounded variables.

    """
    ordered_symbols = [s for _, s in sorted((str(s), s) for s in symbols)]

    static_constraints = {}
    for n, c in km.get_constraints_by_symbols(symbols).items():
        if gm.is_symbol(c.expr):
            s  = gm.free_symbols(c.expr).pop()
            fs = gm.free_symbols(c.lower).union(gm.free_symbols(c.upper))
            if len(fs.difference({s})) == 0:
                static_constraints[s] = (float(gm.subs(c.lower, {s: 0})), float(gm.subs(c.upper, {s: 0})))

    static_bounds = np.array([static_constraints[s] for s in ordered_symbols 
                                                    if  s in static_constraints])

    return [s for s in ordered_symbols if s in static_constraints], \
           static_bounds, \
           [s for s in ordered_symbols if s not in static_constraints]


def generate_transition_function(sym_dt, symbols, overrides=None):
    overrides = {} if overrides is None else overrides
    ordered_symbols = [s for (_, s) in sorted((str(s), s) for s in symbols.union(overrides.keys()))]

    f = []
    for s in ordered_symbols:
        if s not in overrides:
            f.append(gm.wrap_expr(s + gm.DiffSymbol(s) * sym_dt))
        else:
            f.append(overrides[s])

    controls = {str(s) for s in sum([list(gm.free_symbols(e)) for e in f], []) if str(s) != str(sym_dt)}
    f_params = [sym_dt] + [gm.Symbol(s) for s in sorted(controls)]
    temp_f   = gm.Matrix([gm.extract_expr(e) for e in f])
    # print(f_params)
    function = gm.speed_up(temp_f, f_params)

    return ordered_symbols, function, f_params
