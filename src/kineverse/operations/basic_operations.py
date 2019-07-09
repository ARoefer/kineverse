from kineverse.operations.operation import Operation


class CreateSingleValue(Operation):
    def __init__(self, path, value):
        super(CreateSingleValue, self).__init__('Create Single Value: {}'.format(type(value)), [], ['path'], path=path)
        self.value = value

    def _apply(self, ks, path):
        ks.set_data(path, self.value)

def collect_paths(obj, root):
    out = {root}
    for a in [a for a in dir(obj) if a[0] != '_' and not callable(getattr(obj, a))]:
        out.update(collect_paths(getattr(obj, a), root + (a,)))
    return out

class CreateComplexObject(Operation):
    def __init__(self, path, obj):
        super('Create Complex Object: {}'.format(type(obj)), [], list(collect_paths(obj, (path,))), path=path)
        self.value = value

    def _apply(self, ks, path):
        ks.set_data(path, self.value)


class CallFunctionOperator(Operation):
    def __init__(self, path, fn, **kwargs):
        super('Call Function: {}'.format(fn), [k for f in kwargs.keys()], ['path'], path=path, **kwargs)
        self.fn = fn

    def _apply(self, ks, **kwargs):
        ks.set_data(kwargs['path'], self.fn(**{k: v for k, v in kwargs if k != 'path'}))

