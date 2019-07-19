from kineverse.gradients.gradient_math import GC, GM, spw

atomic_types = set([int, float, bool, str, GC])

symengine_matrix_types = set([spw.sp.DenseMatrix, 
                              spw.sp.ImmutableMatrix, 
                              spw.sp.ImmutableDenseMatrix, 
                              spw.sp.MatrixBase, 
                              spw.sp.MutableDenseMatrix])
matrix_types = {GM}.union(symengine_matrix_types)
symengine_types = set([getattr(spw.sp, x) for x in dir(spw.sp) if type(getattr(spw.sp, x)) == type])