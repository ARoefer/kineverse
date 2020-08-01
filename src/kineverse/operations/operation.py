
class NOT_A_VALUE_TYPE(object):
    pass

NOT_A_VALUE = NOT_A_VALUE_TYPE()

class Operation(object):
    def __init__(self, *construction_args):
        # Store arguments used in the construction of this operation for network serialization.
        self._construction_args = [deepcopy(x) for x in construction_args]
        self.output_paths   = None
        self.full_mod_paths = None
        self.memento        = None

    def collect_dependencies

    def execute(self, km):
        # Reset all outputs
        for o in self.__outputs:
            setattr(self, o, NOT_A_VALUE)
        self.constraints    = NOT_A_VALUE
        self.output_paths   = None 
        self.full_mod_paths = None 

        # Invoke the actual implementation
        self._execute_impl(**args_dict)

        # Sanity check
        missing_outputs = [o for o in self.__outputs if getattr(self, o) == NOT_A_VALUE]
        if len(missing_outputs) > 0:
            raise OperationException('Operation implementation did not yield all specified outputs. Class: {} Missing:\n {}'.format(type(self), ' \n'.join(missing_outputs)))
        if self.constraints is NOT_A_VALUE:
            raise OperationException('Operation implementation did not yield a constraint dict. Class: {}'.format(type(self)))
        # Collect the tree structure of the output
        self.output_paths = {o: collect_paths(getattr(self, o), Path()) for o in self.outputs}
        
        # Generate list of all modified paths by prefixing the write-path assigned to an output
        # to the 
        self.full_mod_paths = sum([list({p + x for x in self.output_paths[o]}) 
                                               for o, p in self.output_path_assignments.items()], [])

    # Writes the data generated in execute() to the model and creates mementos for existing data
    def apply_to_model(self, km, stamp):
        if self.output_paths is None:
            raise OperationException('Dictionary containing output paths is not initialized. Did you execute the operation?')

        self.memento = {o: km.get_data(p, stamp) for o, p in self.output_path_assignments.items() 
                                                          if km.has_data(p, stamp)}
        self.constraint_memento = {n: km.get_constraint(n) for n in self.constraints.keys() 
                                                                 if km.has_constraint(n)}

    # Revokes the application of an operation from a model.
    # Writes the data from the mementos to the model and clears mementos.
    def revoke(self, km):
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

        self.memento = None
        self.constraint_memento = None

    def _execute_impl(self, ks, **kwargs):
        raise NotImplementedError

    def _json_data(self, json_dict):
        json_dict['args'] = self._construction_args

    @classmethod
    def json_factory(cls, args):
        return cls(*args)

class SuchOperation(Operation):

    def _execute_impl(self, st_const, st_flex):
