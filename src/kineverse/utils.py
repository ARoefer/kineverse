import os

import copy  as copy_module
import math
import matplotlib.colors as matcolors
import numpy as np
import kineverse.gradients.gradient_math as gm

from collections                       import namedtuple
from kineverse.time_wrapper            import Time

if gm.cm.SYM_MATH_ENGINE == 'SYMENGINE':
    from kineverse.symengine_types         import symengine_types
else:
    symengine_types = set()

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
    ordered_symbols = [s for (_, s) in sorted((str(s), s) for s in symbols)] # .union(overrides.keys()))]

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


BASE_COLORS = [(35, 95, 100), (355, 95, 91), (261, 100, 100), (194, 95, 91), (123, 100, 100)]#, 
               #(15, 95, 100), (293, 95, 91), (244, 100, 100), (156, 95, 91), (76, 100, 100)]
BASE_COLORS = [np.array(b) * np.array([1.0/360, 1e-2, 1e-2]) for b in BASE_COLORS]
BASE_COLORS = [[b * (1.0, 1.0, f) for f in np.linspace(1.0, 0.5, 3)] for b in BASE_COLORS]

def hsv_to_rgb(h, s, v):
    return matcolors.hsv_to_rgb((h, s, v))

def rgb_to_hex(r, g, b):
    return '#{:02X}{:02X}{:02X}'.format(int(r * 255), int(g * 255), int(b * 255))

def print_return(f):
    def wrap(*args):
        out = f(*args)
        # print(out)
        return out
    return wrap


class ColorGenerator(object):
    def __init__(self, dist=0.2, s_lb=0.6, v_lb=0.3, s=1.0, v=1.0):
        self.counter = 0

    def get_color(self):
        out = BASE_COLORS[self.counter % len(BASE_COLORS)][self.counter // len(BASE_COLORS)]

        self.counter = (self.counter + 1) % (len(BASE_COLORS) * len(BASE_COLORS[0]))

        return hsv_to_rgb(*out)

    def get_color_hex(self):
        return rgb_to_hex(*self.get_color())

class ValueRecorder(object):
    def __init__(self, title, *args):
        super(ValueRecorder, self).__init__()
        self._color_generator = ColorGenerator(v=1.0, s_lb=0.75)
        self.title    = title
        group_colors  = [args[x] if type(args[x]) == tuple else (args[x], self._color_generator.get_color_hex()) for x in range(len(args))]
        self.data     = {a: [] for a, _ in group_colors}
        self.data_lim = {a: (1e20, -1e20) for a, _ in group_colors}
        self.colors   = dict(group_colors)
        self.thresholds = {}
        self.x_labels = None
        self.x_title  = None
        self.x_space  = None
        self.y_labels = None
        self.y_title  = None
        self.y_space  = None
        self.grid     = False
        self.legend_loc = None
        self.marker   = None

    def set_marker(self, marker):
        self.marker = marker

    def set_xspace(self, min, max):
        self.x_space  = (min, max)

    def set_xtitle(self, title):
        self.x_title  = title

    def set_xlabels(self, labels):
        self.x_labels = labels

    def set_yspace(self, min, max):
        self.y_space  = (min, max)

    def set_ytitle(self, title):
        self.y_title  = title

    def set_ylabels(self, labels):
        self.y_labels = labels

    def set_grid(self, grid):
        self.grid = grid

    def set_legend_location(self, loc):
        self.legend_loc = loc

    def log_data(self, group, value):
        if group not in self.data:
            raise Exception('Unknown data group "{}"'.format(group))
        l, u = self.data_lim[group]
        self.data_lim[group] = (min(value, l), max(value, u))
        self.data[group].append(value)

    def compute_limits(self):
        for a, d in self.data.items():
            self.data_lim[a] = (min(d), max(d))

    def plot(self, ax):
        if len(self.data_lim) == 0:
            return

        no_xticks = self.x_labels is not None and len(self.x_labels) == 0

        if len(self.data) > 0:
            labels = range(len(list(self.data.values())[0])) if self.x_labels is None or no_xticks else self.x_labels
        else:
            labels = []

        if self.marker is None:
            self.patches = [ax.plot(labels, d, color=self.colors[n], label=n)[0] for n, d in sorted(self.data.items())]
        else:
            self.patches = [ax.plot(labels, d, color=self.colors[n], label=n, marker=self.marker)[0] for n, d in sorted(self.data.items())]

        data_lim_y = (float(min([l[0] for l in self.data_lim.values()])), float(max([l[1] for l in self.data_lim.values()])))

        for n, (y, c) in self.thresholds.items():
            ax.axhline(y, color=c)

        if self.x_space is not None:
            ax.set_xlim(self.x_space)

        if self.y_space is not None:
            ax.set_ylim(self.y_space)
        else:
            y_width = data_lim_y[1] - data_lim_y[0]
            new_width = (data_lim_y[0] - y_width * 0.1, data_lim_y[1] + y_width * 0.1)
            ax.set_ylim(new_width)

        if self.y_labels is not None:
            if type(self.y_labels) is tuple:
                ax.set_yticks(self.y_labels[0])
                ax.set_yticklabels(self.y_labels[1])
            else:
                ax.set_yticks(np.linspace(data_lim_y[0], data_lim_y[1], len(self.y_labels)))
                ax.set_yticklabels(self.y_labels)

        if self.x_title is not None:
            ax.set_xlabel(self.x_title)

        if self.y_title is not None:
            ax.set_ylabel(self.y_title)

        if no_xticks:
            ax.set_xticklabels([])

        loc = 'best' if self.legend_loc is None else self.legend_loc
        ax.legend(handles=self.patches, loc=loc)
        if self.title is not None:
            ax.set_title(self.title)
        ax.grid(self.grid)

    def add_threshold(self, name, value, color=None):
        if color is None:
            color = BASE_COLORS[len(self.thresholds) % len(BASE_COLORS)]
        self.thresholds[name] = (value, color)


class SymbolicRecorder(ValueRecorder):
    def __init__(self, title, **kwargs):
        super(SymbolicRecorder, self).__init__(title, *kwargs.keys())
        self.symbols = kwargs
        self._labels, self._expressions = zip(*self.symbols.items()) if len(self.symbols) > 0 else ([], [])

        temp_matrix = gm.Matrix(self._expressions)
        self._expr_matrix = gm.speed_up(temp_matrix, gm.free_symbols(temp_matrix))

    def log_symbols(self, subs_table):
        np_matrix = self._expr_matrix(**{str(s): v for s, v in subs_table.items()})

        for k, v in zip(self._labels, np_matrix.flatten()):
            self.log_data(k, v)
