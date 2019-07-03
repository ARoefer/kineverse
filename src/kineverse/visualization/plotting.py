import matplotlib.pyplot as plt

from collections import namedtuple
from math import ceil, sqrt

COLORS = ['r', 'g', 'b', 'c', 'm', 'y', 'k', 'w']

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
        self.title    = title
        group_colors  = [args[x] if type(args[x]) == tuple else (args[x], COLORS[x % len(COLORS)]) for x in range(len(args))]
        self.data     = {a: [] for a, _ in group_colors}
        self.data_lim = {a: (1e20, -1e20) for a, _ in group_colors}
        self.colors   = dict(group_colors)

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
        self.patches = [ax.plot(d, self.colors[n], label=n)[0] for n, d in self.data.items()]
        ax.legend(handles=self.patches, loc='center right')
        ax.set_title(self.title)


class SymbolicRecorder(ValueRecorder):
    def __init__(self, title, **kwargs):
        super(SymbolicRecorder, self).__init__(title, *kwargs.keys())
        self.symbols = kwargs

    def log_symbols(self, subs_table):
        for k, s in self.symbols.items():
            self.log_data(k, s.subs(subs_table))