# import symengine as se
#
# symengine_matrix_types = set([se.DenseMatrix,
#                               se.ImmutableMatrix,
#                               se.ImmutableDenseMatrix,
#                               se.MatrixBase,
#                               se.MutableDenseMatrix])
#
# symengine_types  = set([getattr(se.lib.symengine_wrapper, x) for x in dir(se.lib.symengine_wrapper) if type(getattr(se.lib.symengine_wrapper, x)) == type])
# symengine_floats = {se.RealDouble, se.RealNumber}
symengine_floats = set()
symengine_types = set()
symengine_matrix_types = set()
