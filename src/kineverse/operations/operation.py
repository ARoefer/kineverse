from symengine import Symbol

class Path(tuple):
    def __new__(cls, path):
        if type(path) == str:
            return super(Path, cls).__new__(Path, path.split('/'))
        return super(Path, cls).__new__(Path, path)

    def __add__(self, other):
        return Path(super(Path, self).__add__(other))

    def __radd__(self, other):
        return Path(other) + self

    def __getitem__(self, idx):
        if type(idx) is int:
            return super(Path, self).__getitem__(idx)
        return Path(super(Path, self).__getitem__(idx))

    def __eq__(self, other):
        if type(other) is str:
            return self == Path(other)
        return super(Path, self).__eq__(other)

    def __ne__(self, other):
        return not self.__eq__(other)

    def to_symbol(self):
        return Symbol('__'.join(self))

    def __str__(self):
        return '/'.join(self)

    def __repr__(self):
        return 'P{}'.format(super(Path, self).__repr__())

# Every action needs to declare the fields it depends on and the ones it modifies
class Operation(object):
    def __init__(self, name, args, modifications, **kwargs):
        self.args_paths = {a : kwargs[a] for a in args}
        for k, v in kwargs.items():
            if type(v) is not Path and type(v) is not str and k in modifications:
                raise Exception('Assignments of modification parameters need to be Paths.\n Parameter {} is assigned a value of type {}.\n Value: {}'.format(k, type(v), v))
        self.mod_paths  = {m : kwargs[m] if type(kwargs[m]) is Path else Path(kwargs[m]) for m in modifications}
        self._memento   = {} # Memento of the modified expressions

    def apply(self, ks):
        self._memento = {m: ks.get_data(p) for m, p in self.mod_paths.items() if ks.has_data(p)}
        to_write = self._apply(ks, **{k: ks.get_data(e) if type(e) == Path else e for k, e in self.args_paths.items()})
        for k, p in self.mod_paths.items():
            if k not in to_write:
                raise Exception('Implementations of _apply() need to return a dictionary of results specified by the operator\'s signature.\n Operation "{}" is supposed to return "{}", implementation only returned "{}"'.format(self.name, 
                                          ', '.join(sorted(self.mod_paths.keys())), 
                                          ', '.join(sorted(str(x) for x in to_write.keys()))))
            ks.set_data(p, to_write[k])

    def _apply(self, ks, **kwargs):
        raise NotImplementedError

    def revoke(self, ks):
        for p, e in self._memento.values():
            ks.set_data(p, e)
        for p in {p for k, p in self.mod_paths.items() if k not in self._memento}:
            ks.remove_data(p)