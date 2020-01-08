#!/usr/bin/env python
import random
import time

from tqdm import tqdm

from kineverse.gradients.gradient_math import spw, frame3_rpy, point3, Position
from kineverse.visualization.plotting  import ValueRecorder, draw_recorders



def P(n, x, i):
    return Position('{}_{}_{}'.format(n, x, i))

if __name__ == '__main__':

    iterations = 200

    rec_d = ValueRecorder('Evaluation Speed Comparison', ('direct', 'b'), ('llvm', 'm'))
    x_labels = []

    for x in tqdm(range(1, 101)):
        y = x * 1 #10
        frames = [frame3_rpy(P('rr', x, i), P('rp', x, i), P('ry', x, i), point3(P('x', x, i), P('y', x, i), P('z', x, i))) for i in range(y)]
        

        matrix = frames[0]
        for f in frames:
            matrix = matrix.col_join(f)

        x_labels.append(len(matrix.free_symbols))

        cython_m = spw.speed_up(matrix, matrix.free_symbols, 'llvm')

        avg_d = 0.0
        avg_l = 0.0

        for i in range(iterations):
            state = {s: random.random() for s in matrix.free_symbols}

            start = time.time()
            matrix.subs(state)
            avg_d += time.time() - start


            start = time.time()
            cython_m(**{str(s): v for s, v in state.items()})
            avg_l += time.time() - start


        rec_d.log_data('direct', avg_d / iterations)
        rec_d.log_data('llvm', avg_l / iterations)

    rec_d.set_xspace(x_labels[0], x_labels[-1])
    rec_d.set_xtitle('Number of Variables')
    rec_d.set_ytitle('Time (s)')
    draw_recorders([rec_d], ratio=1, plot_width=8, plot_height=5).savefig('llvm_speedtest.png')

    