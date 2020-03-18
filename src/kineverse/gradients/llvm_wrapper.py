# -------------------------------------------------------------
# This code is originally from an old version of giskardpy.
# Project page: https://github.com/SemRoCo/giskardpy
# -------------------------------------------------------------
import numpy     as np
import symengine as se
from symengine.lib.symengine_wrapper import Lambdify

class SymengineException(Exception):
    pass

class CompiledFunction(object):
    def __init__(self, str_params, fast_f, l, shape):
        self.str_params = str_params
        self.fast_f = fast_f
        self.l = l
        self.shape = shape

    def __call__(self, **kwargs):
        try:
            filtered_args = [kwargs[k] for k in self.str_params]
            out = np.empty(self.l)
            self.fast_f.unsafe_real(np.array(filtered_args, dtype=np.double), out)
            return np.nan_to_num(out).reshape(self.shape)
        except KeyError as e:
            raise SymengineException('Parameter "{}" not found in passed kwargs. Try deleting the last loaded compiler to trigger recompilation. Passed kwargs:\n  {}'.format(e.message, '\n  '.join(['{}: {}'.format(k, v) for k, v in kwargs.items()])))

@profile
def speed_up(function, parameters):
    str_params = [str(x) for x in parameters]
    if len(parameters) == 0:
        try:
            constant_result = np.array(function).astype(float).reshape(function.shape)
        except:
            return

        def f(**kwargs):
            return constant_result
        return f
    else:
        try:
            fast_f = Lambdify(list(parameters), function, backend='llvm', cse=True, real=True)
            return CompiledFunction(str_params, fast_f, len(function), function.shape)
        except RuntimeError as e:
            raise Exception('WARNING RuntimeError: "{}" during lambdify with LLVM backend.'.format(e))
