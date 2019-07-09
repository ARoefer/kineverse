from kineverse.operations.data_tree import DataTree
from kineverse.operations.history   import History, Chunk

class KinematicState(object):
    def __init__(self):
        self.data_tree = DataTree()
        self.state = {}
        self.grounded_symbols  = set()
        self.operation_history = History()
        self.timeline_tags     = {}

    def apply_operation(self, op, tag):
        time = self.timeline_tags[tag] if tag in self.timeline_tags else self.operation_history.get_time_stamp()
        while len(self.operation_history.dirty_chunks) > 0 and self.operation_history.dirty_chunks[0].stamp <= time:
            chunk = self.operation_history.dirty_chunks[0]
            chunk.operation.apply(self)
            self.operation_history.flag_clean(chunk)
            self.operation_history.flag_dirty(chunk.dependents)

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

    def clean_structure(self):
        while len(self.operation_history.dirty_chunks) > 0:
            chunk = self.operation_history.dirty_chunks[0]
            chunk.operation.apply(self)
            self.operation_history.flag_clean(chunk)
            self.operation_history.flag_dirty(chunk.dependents)            

    def has_data(self, key):
        if type(key) == str:
            key = tuple(key.split('/'))
        return key in self.data_tree

    def get_data(self, key):
        if type(key) == str:
            key = tuple(key.split('/'))
        return self.data_tree[key]

    def set_data(self, key, value):
        if type(key) == str:
            key = tuple(key.split('/'))
        self.data_tree[key] = value

    def remove_data(self, key):
        if type(key) == str:
            key = tuple(key.split('/'))
        self.data_tree.remove_data(key)

    def set_symbol_value(self, symbol, value=0.0):
        self.state[symbol] = value

    def get_expressions_at(self, time_idx_or_tag):
        pass
        # Collect all chunks 1 after idx/tag
        # Create copy of this state and call chunks' revoke() function on the copy