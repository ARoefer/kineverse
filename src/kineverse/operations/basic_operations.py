from kineverse.model.paths             import Path, collect_paths
from kineverse.operations.operation    import Operation, op_construction_wrapper


class CreateSingleValue(Operation):
    def __init__(self, path, value):
        super(CreateSingleValue, self).__init__('Create Single Value: {}'.format(type(value)), ['path'], path=path)
        self.value = value

    def _apply(self, ks):
        return {'path': self.value}, {}
        

class CreateComplexObject(Operation):
    def __init__(self, path, obj):
        op_construction_wrapper(super(CreateComplexObject, self).__init__,
                                'Create Complex Object: {}'.format(type(obj)), 
                                [], (path, 'path', obj))
        self.obj = obj

    def _apply(self, ks):
        return {'path': self.obj}, {}

class CallFunctionOperator(Operation):
    def __init__(self, path, fn, **kwargs):
        if not hasattr(fn, 'func_code'):
            raise Exception('fn does not seem to be a function')

        args    = fn.func_code.co_varnames[:fn.func_code.co_argcount]
        n_def   = len(fn.func_defaults) if fn.func_defaults is not None else 0
        missing = [args[x] for x in range(len(args) - n_def) if args[x] not in kwargs]
        if len(missing) > 0:
            raise Exception('Arguments "{}" are required by function "{}" but not given to the operation wrapper'.format(', '.join(missing), fn.func_name))
        super(CallFunctionOperator, self).__init__('Call Function: {}'.format(fn), ['path'], path=path, **kwargs)
        self.args_paths = {a: kwargs[a] for a in args if a in kwargs}
        self.fn = fn

    def _apply(self, ks, **kwargs):
        return {'path': self.fn(**{k: v for k, v in kwargs.items() if k != 'path'})}, {}

