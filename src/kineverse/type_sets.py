import kineverse.gradients.common_math as cm

from kineverse.gradients.gradient_math   import GC, GM
import numpy as np

atomic_types = set([int, float, bool, str, GC, type(None), set])

matrix_types = {GM}.union(cm.matrix_types)
symbolic_types = {GC, GM}.union(cm.math_types)
numpy_types = {getattr(np, a) for a in dir(np) if type(getattr(np, a)) == type}

is_symbolic = cm.is_symbolic