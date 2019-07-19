from kineverse.gradients.gradient_math import spw
from kineverse.type_sets               import atomic_types, matrix_types, symengine_types

class PathException(Exception):
    def __init__(self, path, obj):
        super(PathException, self).__init__('Object {} at {} has no attribute {}.'.format(path[:-1], obj, path[-1]))
        self.path = path
        self.obj  = obj


class Path(tuple):
    def __new__(cls, path):
        if type(path) == str:
            return super(Path, cls).__new__(Path, path.split('/'))
        elif type(path) == spw.Symbol:
            return super(Path, cls).__new__(Path, str(path).split('__'))
        return super(Path, cls).__new__(Path, path)

    def __add__(self, other):
        return Path(super(Path, self).__add__(other))

    def __radd__(self, other):
        return Path(other) + self

    def __getitem__(self, idx):
        if type(idx) is int:
            return super(Path, self).__getitem__(idx)
        return Path(super(Path, self).__getitem__(idx))

    def __getslice__(self, i, j):
        return Path(super(Path, self).__getslice__(i, j))

    def __eq__(self, other):
        if type(other) is str:
            return self == Path(other)
        return super(Path, self).__eq__(other)

    def __ne__(self, other):
        return not self.__eq__(other)

    def to_symbol(self):
        return spw.Symbol('__'.join(self))

    def __str__(self):
        return '/'.join(self)

    def __repr__(self):
        return 'P{}'.format(super(Path, self).__repr__())

    def get_data(self, obj):
        try:
            for x in range(len(self)):
                if type(obj) is dict:
                    obj = obj[self[x]]
                elif type(obj) is list:
                    obj = obj[int(self[x])]
                else:
                    obj = getattr(obj, self[x])
            return obj      
        except (KeyError, IndexError, AttributeError):
            raise PathException(self[:x + 1], obj)


stopping_set = atomic_types.union(matrix_types).union(symengine_types)

def collect_paths(obj, root):
    t = type(obj)
    out = {root}
    if t not in stopping_set:
        for a in [a for a in dir(obj) if a[0] != '_' and not callable(getattr(obj, a))]:
            out.update(collect_paths(getattr(obj, a), root + Path(a,)))
    return out

def find_common_root(paths):
    out = Path([])
    if len(paths) > 0:
        idx = 0 
        ok  = True
        while ok:
            part = None
            for p in paths:
                part = p[idx] if part is None and len(p) > idx else part
                if len(p) == idx or p[idx] != part:
                    ok = False
                    break
            if ok:
                out = out + (part, )
                idx += 1
    return out

def is_prefix(a, b):
       return len(a) <= len(b) and b[:len(a)] == a

class PathDict(dict):
    def __init__(self, value=None, paths=[]):
        super(PathDict, self).__init__()
        self.value = value
        for p, v in paths:
            self[p] = v

    # True contains
    def __contains__(self, key):
        return len(key) == 0 or (super(PathDict, self).__contains__(key[0]) and key[1:] in super(PathDict, self).__getitem__(key[0]))

    # Gets closest value!
    def __getitem__(self, key):
        return self.value if len(key) == 0 or not super(PathDict, self).__contains__(key[0]) else super(PathDict, self).__getitem__(key[0])[key[1:]]

    # Gets closest value and returns its key too
    def get_with_key(self, key):
        if len(key) > 0:
            k, v = t.get_with_key(key[1:])
            return key[:1] + k, v
        return tuple(), self.value

    # Sets a value while creating the path to it
    def __setitem__(self, key, value):
        if len(key) == 0:
            self.value = value
        else:
            if not super(PathDict, self).__contains__(key[0]):
                super(PathDict, self).__setitem__(key[0], PathDict())
            super(PathDict, self).__getitem__(key[0])[key[1:]] = value


class PathSet(set):
    # Tests whether a path or any of its prefixes are covered by this set
    def __contains__(self, key):
        for x in range(len(key)):
            if super(PathSet, self).__contains__(key[:x + 1]):
                return True
        return False

    # Tests whether the given key is a prefix of any element in the set
    def is_prefix(self, key):
        for p in self:
            if p[:len(key)] == key:
                return True
        return False

    # Test whether the exact key is in this set
    def true_in(self, key):
        return super(PathSet, self).__contains__(key)

    def add(self, key):
        if type(key) != tuple:
            raise Exception('Path sets can only store tuples. Key: "{}" is of type {}.'.format(key, type(key)))
        super(PathSet, self).add(key)