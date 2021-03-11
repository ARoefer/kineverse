import kineverse.model.model_settings  as model_settings

from kineverse.model.paths import Path, CPath, PathDict, collect_paths
from kineverse.utils       import copy, deepcopy

class OperationException(Exception):
    """Exception thrown when something goes wrong with an operation."""
    pass

class NOT_A_VALUE_TYPE(object):
    pass

NOT_A_VALUE = NOT_A_VALUE_TYPE()

def check_function_signature(fn, args_dict, base_fn=None, superset=False):
    """Checks that a given dictionary contains all necessary arguments to execute a function,
    The function can check whether the dict provides a superset of arguments, or an exact match. It can also strip arguments from consideration based on a base 
    implementation of the function.

    Returns the arguments required by the function.
    """

    args = set(fn.__code__.co_varnames[:fn.__code__.co_argcount])

    if base_fn is not None:
        args -= set(base_fn.__code__.co_varnames[:base_fn.__code__.co_argcount])

    given_args = set(args_dict.keys())
    if len(args) > len(given_args) or (len(args) != len(given_args) and not superset):
        raise OperationException('Argument dict given to operation does not match the arguments required by implementation. Required are {} arguments given were {}'.format(len(args), len(given_args)))
    elif len(args.difference(given_args)) > 0: # Note: Asymmetric difference
        raise OperationException('Argument dict given to operation is missing arguments required by the implementation. Missing:\n {}'.format('\n '.join(sorted(args.difference(given_args)))))

    return args


class Operation(object):
    _output_blacklist = {'_outputs', 
                         'output_path_assignments',
                         'output_paths',
                         'full_mod_paths',
                         'memento',
                         'deep_memento',
                         '_exec_args',
                         'dependencies',
                         'constraints',
                         'constraint_memento'}

    def __init__(self, output_paths, **exec_args):
        # Store arguments used in the construction of this operation for network serialization.
        # self._construction_args = [deepcopy(x) for x in construction_args]
        illegal_outputs = [k for k in output_paths.keys() if k in Operation._output_blacklist]
        if len(illegal_outputs) > 0:
            raise OperationException('Declared outputs would interfer with normal attributes. Outputs in question:\n {}'.format('\n '.join(sorted(illegal_outputs))))
        
        self._outputs = output_paths.keys()

        self.output_path_assignments = {k: v if type(v) == Path else Path(v) for k, v in output_paths.items()}
        self.output_paths   = None
        self.full_mod_paths = None
        self.memento        = None
        self.deep_memento   = None
        self.constraints    = None
        args = set(self._execute_impl.__code__.co_varnames[1:self._execute_impl.__code__.co_argcount])
        given_args = set(exec_args.keys())

        self._exec_args       = {k: deepcopy(exec_args[k]) for k in check_function_signature(self._execute_impl, 
                                                                                             exec_args, 
                                                                                             Operation._execute_impl)}
        self.dependencies = {d for d in self._exec_args.values() if type(d) == Path}


    def execute(self, km, stamp):
        # Reset all outputs
        for o in self._outputs:
            setattr(self, o, NOT_A_VALUE)
        self.constraints    = NOT_A_VALUE
        self.output_paths   = None 
        self.full_mod_paths = None 

        exec_args = {}
        for k, d in self._exec_args.items():
            if type(d) == Path:
                if km.has_data(d, stamp):
                    exec_args[k] = km.get_data(d, stamp)
                else:
                    raise OperationException('Data {} required by operation "{}"" is not available at time {}'.format(d, type(self), stamp))
            else:
                exec_args[k] = d

        # Invoke the actual implementation
        self._execute_impl(**exec_args)

        # Sanity check
        missing_outputs = [o for o in self._outputs if getattr(self, o) == NOT_A_VALUE]
        if len(missing_outputs) > 0:
            raise OperationException('Operation implementation did not yield all specified outputs. Class: {} Missing:\n {}'.format(type(self), ' \n'.join(missing_outputs)))
        if self.constraints is NOT_A_VALUE:
            raise OperationException('Operation implementation did not yield a constraint dict. Class: {}'.format(type(self)))
        # Collect the tree structure of the output
        self.output_paths = {o: collect_paths(getattr(self, o), Path('')) for o in self._outputs}
        
        # Generate list of all modified paths by prefixing the write-path assigned to an output
        # to the 
        self.full_mod_paths = set(sum([[p] + [p + x for x in self.output_paths[o]]
                                                    for o, p in self.output_path_assignments.items()], []))

    # Writes the data generated in execute() to the model and creates mementos for existing data
    def apply_to_model(self, km, stamp):
        if self.output_paths is None:
            raise OperationException('Dictionary containing output paths is not initialized. Did you execute the operation?')

        if model_settings.BRUTE_MODE:
            self.memento = {}
            self.deep_memento = {}
            self.constraint_memento = {}
        else:
            self.memento = {o: km.get_data(p, stamp) for o, p in self.output_path_assignments.items() 
                                                              if km.has_data(p, stamp)}
            self.deep_memento = {}
            for o, d in self.memento.items():
                level = self.deep_memento
                # Add dictionaries for the prefix
                for x in self.output_path_assignments[o][:-1]:
                    if x not in level:
                        level[x] = {}
                    level = level[x]
                level[self.output_path_assignments[o][-1]] = d

            self.constraint_memento = {n: km.get_constraint(n) for n in self.constraints.keys() 
                                                                     if km.has_constraint(n)}
        # Write new data to model
        for o, p in self.output_path_assignments.items():
            km.set_data(p, getattr(self, o))

        for n, c in self.constraints.items():
            km.add_constraint(n, c)

    # Revokes the application of an operation from a model.
    # Writes the data from the mementos to the model and clears mementos.
    def revoke(self, km):
        if not model_settings.BRUTE_MODE:
            if self.memento is None:
                raise OperationException('Cannot revoke operation that has not been applied.')

            for o, p in self.output_path_assignments.items():
                if o in self.memento:
                    km.set_data(p, self.memento[o])
                else:
                    km.remove_data(p)

            for n in self.constraints.keys():
                if n in self.constraint_memento:
                    km.add_constraint(n, self.constraint_memento[n])
                else:
                    km.remove_constraint(n)

        self.memento            = None
        self.deep_memento       = None
        self.constraint_memento = None

    def get_from_memento(self, key):
        return key.get_data_no_throw(self.deep_memento)

    # def _collect_deps(self, **kwargs):
    #     return set()

    def _execute_impl(self, **kwargs):
        raise NotImplementedError

    def _json_data(self, json_dict):
        raise NotImplementedError
        # json_dict['args'] = self._construction_args

    @classmethod
    def json_factory(cls, args):
        raise NotImplementedError
        # return cls(*args)
