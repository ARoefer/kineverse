import matplotlib.pyplot as plt
import matplotlib.colors as matcolors
import pandas            as pd
import numpy             as np

from collections import namedtuple
from math import ceil, sqrt

COLORS = ['r', 'g', 'b', 'c', 'm', 'y', 'k']

def hsv_to_rgb(h, s, v):
    return matcolors.hsv_to_rgb((h, s, v))

def print_return(f):
    def wrap(*args):
        out = f(*args)
        print(out)
        return out
    return wrap


class ColorGenerator(object):
    def __init__(self, dist=0.2, s_lb=0.6, v_lb=0.3, s=1.0, v=1.0):
        self.dist = dist
        self.h    = 0.0
        self.s    = s
        self.v    = v
        self.s_lb = s_lb
        self.v_lb = v_lb
        self._current_step = 1.0 / ceil(1.0 / dist)

    def get_color(self):
        out = hsv_to_rgb(self.h, self.s, self.v)
        self.h += self._current_step
        if self.h >= 1.0:
            self.h = 0
            self.s -= self.dist / self.v
            if self.s < self.s_lb:
                self.s = 1.0
                self.v -= self.dist
                if self.v < self.v_lb:
                    self.v = 1.0
                    self.s = 1.0
                    self.h = 0.0
                    #raise Exception('Can not generate any more colors.')
            self._current_step = self.s * self.v / ceil(self.s * self.v / self.dist)

        return out

    def get_color_hex(self):
        r, g, b = self.get_color()
        return '#{:02X}{:02X}{:02X}'.format(int(r * 255), int(g * 255), int(b * 255))


def draw_recorders(recorders, ratio=1.0, plot_width=3, plot_height=2, sizes=[]):
    #max_w = max([t[0] for t in sizes if t != None])
    #max_h = max([t[1] for t in sizes if t != None])
    a = len(recorders) * plot_width * plot_height #sum([t[0] * t[1] if t != None else plot_width * plot_height for t in sizes])
    w = sqrt(ratio * a)
    cols = int(ceil(w / plot_width))
    rows = int(ceil(len(recorders) / float(cols)))

    gridsize  = (rows, cols)
    plot_size = (plot_width, plot_height)
    fig       = plt.figure(figsize=(cols*plot_size[0], rows*plot_size[1]))
    axes      = [plt.subplot2grid(gridsize, (x / cols, x % cols), colspan=1, rowspan=1) for x in range(len(recorders))]
    for a, r in zip(axes, recorders):
        r.plot(a)
    fig.tight_layout()
    return fig

def convert_qp_builder_log(qp_builder, constraints=[]):
    color_gen = ColorGenerator()

    if len(constraints) == 0:
        constraints = qp_builder.row_names

    dfs_H, dfs_A, cmds = qp_builder.H_dfs, qp_builder.A_dfs, qp_builder.cmd_df
    lbs     = pd.DataFrame([df.T['lb'].T for df in dfs_H]).reset_index(drop=True)
    ubs     = pd.DataFrame([df.T['ub'].T for df in dfs_H]).reset_index(drop=True)
    weights = pd.DataFrame([df.T['weight'].T for df in dfs_H]).reset_index(drop=True)

    colors = {}
    constraint_recs = {}
    for cn in constraints:
        if cn in qp_builder.row_names:
            cs  = pd.DataFrame([df.T[cn].T for df in dfs_A]).reset_index(drop=True)
            names = [c for c in cs.columns if (cs[c] != 0.0).any()]
            for n in names:
                if n not in colors:
                    colors[n] = color_gen.get_color_hex()

            rec = ValueRecorder(cn, *[(n, colors[n]) for n in names])
            rec.data = {c: cs[c] for c in rec.data.keys()}
            constraint_recs[cn] = rec

    rec_w = ValueRecorder('weights', *[(c, colors[c]) for c in weights.columns if c in colors])
    rec_w.data = {c: weights[c] for c in rec_w.data.keys()}

    rec_b = ValueRecorder('bounds', *sum([[('{}_lb'.format(c), colors[c]), ('{}_ub'.format(c), colors[c])] for c in lbs.columns if c in colors], []))
    rec_b.data = dict(sum([[('{}_lb'.format(c), lbs[c]), ('{}_ub'.format(c), ubs[c])] for c in lbs.columns if c in colors], []))

    rec_c = ValueRecorder('commands', *[(c, colors[c]) for c in cmds.columns if c in colors])
    rec_c.data = {c: cmds[c] for c in rec_c.data.keys()}

    return rec_w, rec_b, rec_c, constraint_recs


class PlotProp(object):
    def __init__(self, name, lower, upper):
        self.name   = name
        self.lower  = lower
        self.upper  = upper 
        self.height = upper - lower

class PlotGroup(object):
    def __init__(self, plots, median=None, height=None):
        self.plots = plots
        self.height = height if height is not None else max([p.upper for p in plots]) - min([p.lower for p in plots])
        self.median = median if median is not None else min([p.lower for p in plots]) + 0.5 * self.height

    def recompute(self):
        self.height = max([p.upper for p in self.plots]) - min([p.lower for p in self.plots])
        self.median = min([p.lower for p in self.plots]) + 0.5 * self.height        


def split_recorders(recorders, threshold=0.1, flatline=1e-4):
    out = []
    for r in recorders:
        if len(r.data_lim) == 0:
            out.append(r)
            continue

        g = PlotGroup([PlotProp(a, l, u) for a, (l, u) in r.data_lim.items()])

        groups = [g]
        stop   = False
        x      = 0
        while not stop:
            stop = True
            while x < len(groups):
                g = groups[x]
                data_mean = sum([p.lower + 0.5 * p.height for p in g.plots]) / len(g.plots)
                g.plots.sort(key=lambda p: abs(p.lower + 0.5 * g.height - data_mean), reverse=True)
                y = 0
                while y < len(g.plots):
                    p = g.plots[y]
                    if p.height > flatline and p.height / g.height < threshold:
                        new_group = True
                        for h in groups:
                            nl = min(h.median - 0.5 * h.height, p.lower)
                            nu = max(h.median + 0.5 * h.height, p.upper)
                            nh = nu - nl
                            if p.height / nh > threshold and h.height / nh > threshold:
                                h.plots.append(p)
                                h.median = 0.5 * (nl + nu)
                                h.height = nh
                                new_group = False
                                break
                        if new_group:
                            groups.append(PlotGroup([p], p.lower + 0.5 * p.height, p.upper - p.lower))
                        
                        del g.plots[y]
                        g.recompute()
                        stop = False
                    else:
                        y += 1
                x += 1

        if len(groups) > 1:
            for c, g in enumerate(groups):
                nr = ValueRecorder('{} ({})'.format(r.title, c + 1), *[(p.name, r.colors[p.name]) for p in g.plots])
                nr.data     = {p.name: r.data[p.name]     for p in g.plots}
                nr.data_lim = {p.name: (p.lower, p.upper) for p in g.plots}
                out.append(nr)
        else:
            out.append(r)
    return out


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
        if len(self.data) > 0:
            labels = range(len(self.data.values()[0])) if self.x_labels is None else self.x_labels
        else:
            labels = []

        self.patches = [ax.plot(labels, d, color=self.colors[n], label=n)[0] for n, d in sorted(self.data.items())]
        
        for n, (y, c) in self.thresholds.items():
            ax.axhline(y, color=c)

        if self.x_space is not None:
            ax.set_xlim(self.x_space)

        if self.y_space is not None:
            ax.set_ylim(self.y_space)

        if self.y_labels is not None:
            ax.set_yticklabels(self.y_labels)

        if self.x_title is not None:
            ax.set_xlabel(self.x_title)

        if self.y_title is not None:
            ax.set_ylabel(self.y_title)

        ax.legend(handles=self.patches, loc='best')
        ax.set_title(self.title)

    def add_threshold(self, name, value, color=None):
        if color is None:
            color = COLORS[len(self.thresholds) % len(COLORS)]
        self.thresholds[name] = (value, color)


class SymbolicRecorder(ValueRecorder):
    def __init__(self, title, **kwargs):
        super(SymbolicRecorder, self).__init__(title, *kwargs.keys())
        self.symbols = kwargs

    def log_symbols(self, subs_table):
        for k, s in self.symbols.items():
            self.log_data(k, s.subs(subs_table))