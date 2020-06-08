import kineverse.gradients.common_math as cm

from kineverse.gradients.gradient_math   import GC, GM

atomic_types = set([int, float, bool, str, GC])

matrix_types = {GM}.union(cm.matrix_types)
symbolic_types = {GC, GM}.union(cm.math_types)

is_symbolic = cm.is_symbolic