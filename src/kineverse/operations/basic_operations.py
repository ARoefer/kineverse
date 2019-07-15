from kineverse.gradients.gradient_math import GC, GM, spw
from kineverse.model.paths             import Path
from kineverse.operations.operation    import Operation

stopping_set = set([int, float, bool, str, GC, GM] + [getattr(spw.sp, x) for x in dir(spw.sp) if type(getattr(spw.sp, x)) == type])

def collect_paths(obj, root):
    t = type(obj)
    out = {root}
    if t not in stopping_set:
        for a in [a for a in dir(obj) if a[0] != '_' and not callable(getattr(obj, a))]:
            out.update(collect_paths(getattr(obj, a), root + Path(a,)))
    return out


class CreateSingleValue(Operation):
    def __init__(self, path, value):
        super(CreateSingleValue, self).__init__('Create Single Value: {}'.format(type(value)), [], ['path'], path=path)
        self.value = value

    def _apply(self, ks):
        return {'path': self.value}
        

class CreateComplexObject(Operation):
    def __init__(self, path, obj):
        super(CreateComplexObject, self).__init__('Create Complex Object: {}'.format(type(obj)), [], ['path'], path=path)
        self.mod_paths.update({'/'.join(p[1:]) : p  for p in collect_paths(obj, path)})
        self.value = obj

    def apply(self, ks):
        self._memento = {'': ks.get_data(self.mod_paths[''])} if ks.has_data(self.mod_paths['']) else {}
        ks.set_data(self.mod_paths[''], self.value)

    def _apply(self, ks, **kwargs):
        raise Exception('CreateComplexObject() is a hack of the usual operation schema. The _apply function should never be called.')

    def revoke(self, ks):
        for p, e in self._memento.values():
            ks.set_data(p, e)
        if '' not in self._memento:
            ks.remove_data(self.mod_paths[''])


class CallFunctionOperator(Operation):
    def __init__(self, path, fn, **kwargs):
        if not hasattr(fn, 'func_code'):
            raise Exception('fn does not seem to be a function')

        args    = fn.func_code.co_varnames[:fn.func_code.co_argcount]
        n_def   = len(fn.func_defaults) if fn.func_defaults is not None else 0
        missing = [args[x] for x in range(len(args) - n_def) if args[x] not in kwargs]
        if len(missing) > 0:
            raise Exception('Arguments "{}" are required by function "{}" but not given to the operation wrapper'.format(', '.join(missing), fn.func_name))
        super(CallFunctionOperator, self).__init__('Call Function: {}'.format(fn), [a for a in args if a in kwargs], ['path'], path=path, **kwargs)
        self.fn = fn

    def _apply(self, ks, **kwargs):
        return {'path': self.fn(**{k: v for k, v in kwargs.items() if k != 'path'})}

