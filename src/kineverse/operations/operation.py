from kineverse.utils       import copy, deepcopy
from symengine import Symbol
from kineverse.model.paths       import Path, find_common_root, is_prefix, collect_paths
from kineverse.json_serializable import JSONSerializable

def op_construction_wrapper(init_fn, name, mod_list, *written_objs, **kwargs):
    add_kwargs = kwargs
    mod_attrs  = []
    for path, name, obj in written_objs:
        attrs = collect_paths(obj, Path(name))
        add_kwargs.update({str(a): path + a[1:] for a in attrs})
        mod_attrs.extend([str(a) for a in attrs])
    init_fn(name, mod_list + mod_attrs, **add_kwargs)

# Every action needs to declare the fields it depends on and the ones it modifies
class Operation(JSONSerializable):
    def __init__(self, *args):
        self._construction_args = [deepcopy(a) for a in args]
        self.init(*args)

    def init(self, name, modifications, **kwargs):
        self.name = name
        args = self._apply.func_code.co_varnames[2:self._apply.func_code.co_argcount]
        self.args_paths = {a : kwargs[a] for a in args}
        for k, v in kwargs.items():
            if type(v) is not Path and type(v) is not str and k in modifications:
                raise Exception('Assignments of modification parameters need to be Paths.\n Parameter {} is assigned a value of type {}.\n Value: {}'.format(k, type(v), v))
        self.mod_paths  = {m : kwargs[m] if type(kwargs[m]) is Path else Path(kwargs[m]) for m in modifications}
        temp_set = {}
        for m, p in self.mod_paths.items():
            mp = Path(m)
            if mp[0] not in temp_set:
                temp_set[mp[0]] = set()
            temp_set[mp[0]].add(p)

        # Abridged version of mod_paths. Maps joint roots of m to joint roots in p
        self._root_set  = {r: find_common_root(s) for r, s in temp_set.items()}
        # Memento of the modified expressions
        self._memento   = {} 
        self._constraint_memento = {}

    @profile
    def apply(self, ks, touched_set=set()):
        args = {}
        # Cover the case that it can be possible to depend on values which one also modifies
        for k, e in self.args_paths.items():
            if type(e) != Path:                                         # Parameter is constant
                args[k] = e
            elif e in self.mod_paths.values() and e not in touched_set: # Parameter is modified by action an has not been modified during this update process
                for m, p in self._root_set.items():
                    if is_prefix(p, e) and m in self._memento:
                        args[k] = e[len(p):].get_data(self._memento[m])
                        break
                if k not in args:
                    args[k] = ks.get_data(e)
            else: # Parameter is either not modified by this or has already been updated
                args[k] = ks.get_data(e)

        self._memento = {m: deepcopy(ks.get_data(p)) for m, p in self._root_set.items() if ks.has_data(p)}
        to_write, constraints = self._apply(ks, **args)
        # MISSING: VALUE CHECK AGAINST MOD_PATHS! 
        for k, p in self._root_set.items():
            if type(p) != Path:
                raise Exception('Path for modification {} in {} is not a path but a {}!'.format(k, self.name, type(p)))
            if k not in to_write:
                raise Exception('Implementations of _apply() need to return a dictionary of results specified by the operator\'s signature.\n Operation "{}" is supposed to return "{}", implementation only returned "{}"'.format(self.name, 
                                          ', '.join(sorted(self._root_set.keys())), 
                                          ', '.join(sorted(str(x) for x in to_write.keys()))))
            ks.set_data(p, to_write[k])

        for k, c in constraints.items():
            self._constraint_memento[k] = ks.get_constraint(k) if ks.has_constraint(k) else None
            ks.add_constraint(k, c)

    def _apply(self, ks, **kwargs):
        raise NotImplementedError

    @profile
    def revoke(self, ks):
        for m, e in self._memento.items():
            ks.set_data(self._root_set[m], e)
        for p in {p for k, p in self._root_set.items() if k not in self._memento}:
            ks.remove_data(p)
        for k, c in self._constraint_memento.items():
            if c is None:
                ks.remove_constraint(k)
            else:
                ks.add_constraint(k, c)

    def _json_data(self, json_dict):
        json_dict['args'] = self._construction_args

    @classmethod
    def json_factory(cls, args):
        return cls(*args)

    def __eq__(self, other):
        if isinstance(other, type(self)):
            return self.name == other.name and self.args_paths == other.args_paths and self.mod_paths == other.mod_paths
        return False