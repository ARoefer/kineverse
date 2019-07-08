from kineverse.operations.data_tree import DataTree
from kineverse.operations.history   import History

class KinematicState(object):
    def __init__(self):
        self.data_tree = DataTree()
        self.state = {}
        self.grounded_symbols  = set()
        self.operation_history = History()
        self.timeline_tags     = {}

    def apply_operation(self, op, tag):
        time = self.timeline_tags[tag] if tag in self.timeline_tags else self.history.get_time_stamp()
        while len(self.operation_history.dirty_chunks) > 0 and self.operation_history.dirty_chunks[0].stamp <= time:
            chunk = self.operation_history.dirty_chunks[0]
            chunk.op.apply(self)
            self.operation_history.flag_clean(chunk)
            self.operation_history.flag_dirty(chunk.dependents)

        if tag in self.timeline_tags:
            chunk = self.operation_history.get_chunk(self.timeline_tags[tag])
            if chunk is None:
                raise Exception('History returned no chunk for tag {} associated with timestamp {}. This should not happen.'.format(tag, self.timeline_tags[tag]))
            chunk.op.revoke(self)
            self.operation_history.replace_chunk(chunk, Chunk(time, op))
            op.apply(self)
        else:
            self.timeline_tags[tag] = time
            self.operation_history.insert_chunk(Chunk(time, op))
            op.apply(self)

    def clean_structure(self):
        while len(self.operation_history.dirty_chunks) > 0:
            chunk = self.operation_history.dirty_chunks[0]
            chunk.op.apply(self)
            self.operation_history.flag_clean(chunk)
            self.operation_history.flag_dirty(chunk.dependents)            

    def set_value(self, key, value):
        self.data_tree[key] = value

    def set_symbol_value(self, symbol, value=0.0):
        self.state[symbol] = value

    def get_expressions_at(self, time_idx_or_tag):
        pass
        # Collect all chunks 1 after idx/tag
        # Create copy of this state and call chunks' revoke() function on the copy