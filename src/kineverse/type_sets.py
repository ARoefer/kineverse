from kineverse.symengine_types           import symengine_matrix_types, symengine_types
from kineverse.gradients.gradient_math   import GC, GM

atomic_types = set([int, float, bool, str, GC])

matrix_types = {GM}.union(symengine_matrix_types)
symbolic_types = {GC, GM}.union(symengine_types)