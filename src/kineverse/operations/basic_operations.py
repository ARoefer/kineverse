import inspect

from kineverse.utils                import deepcopy
from kineverse.model.paths          import Path, CPath, collect_paths
from kineverse.operations.operation import Operation #, op_construction_wrapper

# class CreateSingleValue(Operation):
#     def init(self, path, value):
#         super(CreateSingleValue, self).init('Create Single Value: {}'.format(type(value)), ['path'], path=path)
#         self.value = value

#     def _apply(self, ks):
#         return {'path': self.value}, {}
        

# class CreateComplexObject(Operation):
#     def init(self, path, obj):
#         op_construction_wrapper(super(CreateComplexObject, self).init,
#                                 'Create Complex Object: {}'.format(type(obj)), 
#                                 [], (path, 'path', obj))
#         self.obj = obj

#     def _apply(self, ks):
#         return {'path': self.obj}, {}

class CreateValue(Operation):
    def __init__(self, path, value):
        super(CreateValue, self).__init__({'output': path}, value=value)
        self._save_serialization_args(path, value)

    def _execute_impl(self, value):
        self.output = deepcopy(value)
        self.constraints = {}


class ExecFunction(Operation):
    def __init__(self, out_path, fn, *fn_args, **constraints):
        if type(fn) == type:
            # Remove the "self" argument

            args    = [k for k in inspect.signature(fn.__init__).parameters.keys()][1:]
            n_def   = len([p for p in inspect.signature(fn.__init__).parameters.values() if p.default != inspect._empty])
            fn_name = str(fn)
        else:
            args    = [k for k in inspect.signature(fn).parameters.keys()]
            n_def   = len([p for p in inspect.signature(fn).parameters.values() if p.default != inspect._empty])
            fn_name = str(fn)
        if len(fn_args) < len(args) - n_def:
            raise Exception('Too few arguments given! Arguments "{}" are required by function "{}" but not given to the operation wrapper'.format(', '.join(args[len(fn_args) - 1:-n_def]), fn_name))
        super(ExecFunction, self).__init__({'result': out_path}, function=fn)
        self._save_serialization_args(out_path, fn, *fn_args, constraints)

        # Manually modify the arguments and dependencies for this operation. DO NOT DO THIS OTHERWISE, IT IS BAD PRACTICE
        self._exec_args.update({k: deepcopy(v) for k, v in zip(args[:len(fn_args)], fn_args)})
        self.dependencies = {d for d in self._exec_args.values() if type(d) == Path}
        self._constraints = constraints

    def _execute_impl(self, function, **kwargs):
        self.result = function(**kwargs)
        self.constraints = self._constraints


# class CallFunctionOperator(Operation):
#     def init(self, path, fn, *params):
#         if not hasattr(fn, '__code__'):
#             raise Exception('fn does not seem to be a function')

#         args    = fn.__code__.co_varnames[:fn.__code__.co_argcount]
#         n_def   = len(fn.func_defaults) if fn.func_defaults is not None else 0
#         if len(params) < len(args) - n_def:
#             raise Exception('Too few arguments given! Arguments "{}" are required by function "{}" but not given to the operation wrapper'.format(', '.join(args[len(params) - 1:-n_def]), fn.func_name))
#         kwargs = dict(zip(args[:len(params)], params))
#         super(CallFunctionOperator, self).init('Call Function: {}'.format(fn), ['path'], path=path, **kwargs)
#         self.args_paths = kwargs
#         self.fn = fn

#     def _apply(self, ks, **kwargs):
#         return {'path': self.fn(**{k: v for k, v in kwargs.items() if k != 'path'})}, {}
