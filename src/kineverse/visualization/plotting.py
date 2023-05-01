import matplotlib.pyplot as plt
import pandas            as pd
import numpy             as np

from collections import namedtuple
from math import ceil, sqrt

from kineverse        import gm
from kineverse.utils  import ValueRecorder, \
                             ColorGenerator
                             
from kineverse.motion import GQPB



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
    axes      = [plt.subplot2grid(gridsize, (x // cols, x % cols), colspan=1, rowspan=1) for x in range(len(recorders))]
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
            rec.compute_limits()

    rec_w = ValueRecorder('weights', *[(c, colors[c]) for c in weights.columns if c in colors])
    rec_w.data = {c: weights[c] for c in rec_w.data.keys()}
    rec_w.compute_limits()

    rec_b = ValueRecorder('bounds', *sum([[('{}_lb'.format(c), colors[c]), ('{}_ub'.format(c), colors[c])] for c in lbs.columns if c in colors], []))
    rec_b.data = dict(sum([[('{}_lb'.format(c), lbs[c]), ('{}_ub'.format(c), ubs[c])] for c in lbs.columns if c in colors], []))
    rec_b.compute_limits()

    rec_c = ValueRecorder('commands', *[(c, colors[c]) for c in cmds.columns if c in colors])
    rec_c.data = {c: cmds[c] for c in rec_c.data.keys()}
    rec_c.compute_limits()

    return rec_w, rec_b, rec_c, constraint_recs


def filter_contact_symbols(recorder, qp_builder):
    if not isinstance(qp_builder, GQPB):
        return recorder

    fields  = [f for f in recorder.data.keys() if max([gm.Symbol(f) in ch.state for ch in qp_builder.collision_handlers.values()] + [False]) is False]
    new_rec = ValueRecorder(recorder.title, *[(f, recorder.colors[f]) for f in fields])
    new_rec.data = {f: recorder.data[f] for f in fields}
    new_rec.data_lim = {f: recorder.data_lim[f] for f in fields}
    new_rec.thresholds = recorder.thresholds.copy()
    new_rec.x_labels   = recorder.x_labels
    new_rec.x_title    = recorder.x_title
    new_rec.x_space    = recorder.x_space
    new_rec.y_labels   = recorder.y_labels
    new_rec.y_title    = recorder.y_title
    new_rec.y_space    = recorder.y_space
    new_rec.grid       = recorder.grid
    new_rec.legend_loc = recorder.legend_loc

    return new_rec


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



