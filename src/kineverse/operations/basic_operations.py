from kineverse.model.paths             import Path, collect_paths
from kineverse.operations.operation    import Operation, op_construction_wrapper


class CreateSingleValue(Operation):
    def init(self, path, value):
        super(CreateSingleValue, self).init('Create Single Value: {}'.format(type(value)), ['path'], path=path)
        self.value = value

    def _apply(self, ks):
        return {'path': self.value}, {}
        

class CreateComplexObject(Operation):
    def init(self, path, obj):
        op_construction_wrapper(super(CreateComplexObject, self).init,
                                'Create Complex Object: {}'.format(type(obj)), 
                                [], (path, 'path', obj))
        self.obj = obj

    def _apply(self, ks):
        return {'path': self.obj}, {}

class CallFunctionOperator(Operation):
    def init(self, path, fn, *params):
        if not hasattr(fn, 'func_code'):
            raise Exception('fn does not seem to be a function')

        args    = fn.func_code.co_varnames[:fn.func_code.co_argcount]
        n_def   = len(fn.func_defaults) if fn.func_defaults is not None else 0
        if len(params) < len(args) - n_def:
            raise Exception('Too few arguments given! Arguments "{}" are required by function "{}" but not given to the operation wrapper'.format(', '.join(args[len(params) - 1:-n_def]), fn.func_name))
        kwargs = dict(zip(args[:len(params)], params))
        super(CallFunctionOperator, self).init('Call Function: {}'.format(fn), ['path'], path=path, **kwargs)
        self.args_paths = kwargs
        self.fn = fn

    def _apply(self, ks, **kwargs):
        return {'path': self.fn(**{k: v for k, v in kwargs.items() if k != 'path'})}, {}

