from kineverse.model.data_tree import DataTree
from kineverse.model.history   import History, Chunk
from kineverse.model.paths     import Path

class Constraint(object):
    def __init__(self, lower, upper, expr):
        self.lower = lower
        self.upper = upper
        self.expr  = expr

class KinematicModel(object):
    def __init__(self):
        self.data_tree         = DataTree()
        self.operation_history = History()
        self.timeline_tags     = {}
        self.constraints       = {}
        self.constraint_symbol_map = {}
        self._touched_set      = set()
        self._touched_stamp    = 0

    def apply_operation(self, op, tag):
        time = self.timeline_tags[tag] if tag in self.timeline_tags else self.operation_history.get_time_stamp()
        self.clean_structure(time)

        if tag in self.timeline_tags:
            chunk = self.operation_history.get_chunk(self.timeline_tags[tag])
            if chunk is None:
                raise Exception('History returned no chunk for tag "{}"" associated with timestamp {}. This should not happen.'.format(tag, self.timeline_tags[tag]))
            chunk.operation.revoke(self)
            self.operation_history.replace_chunk(chunk, Chunk(time, op))
            op.apply(self)
        else:
            self.timeline_tags[tag] = time
            self.operation_history.insert_chunk(Chunk(time, op))
            op.apply(self)

    def remove_operation(self, tag):
        if tag not in self.timeline_tags:
            raise Exception('Tag "{}" not found in time line.'.format(tag))
        chunk = self.operation_history.get_chunk(self.timeline_tags[tag])
        if chunk is None:
            raise Exception('History returned no chunk for tag "{}" associated with timestamp {}. This should not happen.'.format(tag, self.timeline_tags[tag]))
        chunk.operation.revoke(self)
        self.operation_history.remove_chunk(chunk)
        del self.timeline_tags[tag]

    def clean_structure(self, until=2e9):
        while len(self.operation_history.dirty_chunks) > 0 and self.operation_history.dirty_chunks[0].stamp <= until:
            chunk = self.operation_history.dirty_chunks[0]
            if self._touched_stamp > chunk.stamp:
                self._touched_set = set()
            chunk.operation.apply(self, self._touched_set)
            self._touched_set |= chunk.modifications
            self.operation_history.flag_clean(chunk)
            self.operation_history.flag_dirty(*chunk.dependents)            

    def has_data(self, key):
        if type(key) == str:
            key = Path(key.split('/'))
        return key in self.data_tree

    def get_data(self, key):
        if type(key) == str:
            key = Path(key.split('/'))
        return self.data_tree[key]

    def set_data(self, key, value):
        if type(key) == str:
            key = Path(key.split('/'))
        self.data_tree[key] = value

    def remove_data(self, key):
        if type(key) == str:
            key = Path(key.split('/'))
        self.data_tree.remove_data(key)

    def add_constraint(self, key, constraint):
        if key in self.constraints:
            c = self.constraints[key]
            for s in c.expr.free_symbols:
                self.constraint_symbol_map[s].remove(c)    
        self.constraints[key] = constraint
        for s in constraint.expr.free_symbols:
            if s not in self.constraint_symbol_map:
                self.constraint_symbol_map[s] = set()
            self.constraint_symbol_map[s].add(constraint)

    def has_constraint(self, key):
        return key in self.constraints

    def get_constraint(self, key):
        return self.constraints[key]

    def remove_constraint(self, key):
        c = self.constraints[key]
        for s in c.expr.free_symbols:
            self.constraint_symbol_map[s].remove(c)
        del self.constraints[key]

    def get_constraints_by_symbols(self, symbol_set):
        out = set()
        for s in symbol_set:
            if s in self.constraint_symbol_map:
                out.update(self.constraint_symbol_map[s])
        return out 
    def str_op_history(self):
        return '\n'.join(['{:>9.4f}: {}'.format(s, t) for s, t in sorted([(s, t) for t, s in self.timeline_tags.items()])])

    def get_expressions_at(self, time_idx_or_tag):
        pass
        # Collect all chunks 1 after idx/tag
        # Create copy of this state and call chunks' revoke() function on the copy