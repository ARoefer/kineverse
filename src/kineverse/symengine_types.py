import giskardpy.symengine_wrappers as spw

symengine_matrix_types = set([spw.sp.DenseMatrix, 
                              spw.sp.ImmutableMatrix, 
                              spw.sp.ImmutableDenseMatrix, 
                              spw.sp.MatrixBase, 
                              spw.sp.MutableDenseMatrix])

symengine_types = set([getattr(spw.sp.lib.symengine_wrapper, x) for x in dir(spw.sp.lib.symengine_wrapper) if type(getattr(spw.sp.lib.symengine_wrapper, x)) == type])