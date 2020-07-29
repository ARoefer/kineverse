from kineverse.utils       import copy, deepcopy
from symengine import Symbol
from kineverse.model.paths       import Path, find_common_root, is_prefix, collect_paths
from kineverse.json_serializable import JSONSerializable

def op_construction_wrapper(init_fn, name, mod_list, *written_objs, **kwargs):
    """Helper meant to automatically generate the set of modified paths."""
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
        """This function initializes an instantiation of an operation. 
        It collects the set of paths that the operation will read and the set that it will modify.

        :param name: Name of the operation.
        :param modifications: 
        """
        self.name = name
        # list of arguments that need to be passed to the implementation _apply
        args = self._apply.func_code.co_varnames[2:self._apply.func_code.co_argcount]
        # print('{} kwargs:\n{}'.format(self.name, ' \n'.join(sorted(kwargs.keys()))))
        # Retrieve the arguments required by _apply from kwargs
        self.args_paths = {a : kwargs[a] for a in args}
        # Sanity check: If an argument is mentioned in the modifications, its assignment must be a path or string
        for k, v in kwargs.items():
            if type(v) is not Path and type(v) is not str and k in modifications:
                raise Exception('Assignments of modification parameters need to be Paths.\n Parameter {} is assigned a value of type {}.\n Value: {}'.format(k, type(v), v))
        # Compile a dictionary of modifications. {str: Path}
        self.mod_paths  = {m : kwargs[m] if type(kwargs[m]) is Path 
                                         else Path(kwargs[m]) 
                                         for m in modifications}

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
        self._inv_memento = {}
        self._constraint_memento = {}

    @profile
    def apply(self, ks, touched_set=set(), stamp=None):
        args = {}
        # Compile the dict of arguments passed to _apply
        for k, e in self.args_paths.items():
            # Parameter is constant
            if type(e) != Path:
                args[k] = e
            # The parameter has been modified by a previous operation and is 
            # thus currently a valid version.
            elif e in touched_set:
                args[k] = ks.get_data(e)
            else:
                # ASSUMPTION: An operation is only inserted into the history AFTER its 
                # successful application. Thus, IF this is the first execution and the 
                # operation has no memo, it will NOT be queried by the data-retrieval
                args[k] = ks.get_data(e, stamp)
            # elif e in self.mod_paths.values() and : 
            # # Parameter is modified by action this has not been modified during this update process.
            # # This implies, that the parameter is still the result of the last execution of this operation
            # # or a later one. (If this operation was executed once before)
            #     # Try retrieving the data from the previously saved memo
            #     for m, p in self._root_set.items():
            #         if is_prefix(p, e) and m in self._memento:
            #             args[k] = e[len(p):].get_data(self._memento[m])
            #             break
            #     # If data is not contained in memo, this is the first run. Thus it is right to retrieve the data from the model directly.
            #     # FIXME: It seems that this is not correct. Imagine operations A -> B creating a field
            #     # and modifying it. 
            #     # After the model is built once, insert C in the middle, which depends on the newly
            #     # created and modified field. It seems like it would retrieve the version of the field
            #     # that was left by B and not the one left by A.
            #     # CONFIRMED: This is indeed something that happens.
            #     if k not in args:
            #         args[k] = ks.get_data(e)
            # else: # Parameter is either not modified by this or has already been updated
            #     args[k] = ks.get_data(e)

        # Create a memento for the paths being modified, if they exist yet.
        self._memento = {m: deepcopy(args[m]) if m in args else deepcopy(ks.get_data(p, stamp))
                                              for m, p in self._root_set.items()
                                                       if m in args or ks.has_data(p, stamp)}
        
        # Run actual operation implementation
        # Implementation returns a dict mapping the modification-keys to data that is 
        # supposed to be written. Also returns a dict of constraints.
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

        # Memoize existing constraints and write new ones.
        for k, c in constraints.items():
            self._constraint_memento[k] = ks.get_constraint(k) if ks.has_constraint(k) else None
            ks.add_constraint(k, c)

    def _apply(self, ks, **kwargs):
        raise NotImplementedError

    def get_from_memento(self, path):
        if len(self._memento) == 0:
            raise Exception('The memento of operation "{}" is empty.'.format(self.name))

        for m, p in self._root_set.items():
            if is_prefix(p, path):
                return deepcopy(path[len(p):].get_data(self._memento[m]))

        raise Exception('Data for path "{}" is not contained in the memento of this operation'.format(path))

    @profile
    def revoke(self, ks):
        """Revoke the application of this operation on a model."""
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
