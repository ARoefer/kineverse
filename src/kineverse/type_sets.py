from kineverse.gradients.gradient_matrix import symengine_matrix_types
from kineverse.gradients.gradient_math   import GC, GM, spw

atomic_types = set([int, float, bool, str, GC])

matrix_types = {GM}.union(symengine_matrix_types)
symengine_types = set([getattr(spw.sp, x) for x in dir(spw.sp) if type(getattr(spw.sp, x)) == type])