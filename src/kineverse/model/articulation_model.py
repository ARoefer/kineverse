import traceback 
import kineverse.json_wrapper       as json

from kineverse.model.data_tree      import DataTree
from kineverse.model.history        import History, Timeline, StampedData, Chunk
from kineverse.model.paths          import Path
from kineverse.operations.operation import Operation
from kineverse.type_sets            import is_symbolic
from kineverse.json_serializable    import JSONSerializable

Tag = str

class ApplicationInstruction(JSONSerializable):
    @type_check(Operation, Tag)
    def __init__(self, op, tag):
        self.op  = op
        self.tag = tag

    def _json_data(self, json_dict):
        json_dict['op'] = self.op
        json_dict['tag'] = self.tag


class ApplyAt(ApplicationInstruction):
    def __init__(self, op, tag):
        super(ApplyAt, self).__init__(op, tag)

class ApplyBefore(ApplicationInstruction):
    def __init__(self, op, tag, before):
        super(ApplyBefore, self).__init__(op, tag)
        self.ref_tag = before

    def _json_data(self, json_dict):
        super(ApplyBefore, self)._json_data(json_dict)
        json_dict['before'] = self.ref_tag


class ApplyAfter(ApplicationInstruction):
    def __init__(self, op, tag, after):
        super(ApplyAfter, self).__init__(op, tag)
        self.ref_tag = after

    def _json_data(self, json_dict):
        super(ApplyAfter, self)._json_data(json_dict)
        json_dict['after'] = self.ref_tag


class RemoveOp(ApplicationInstruction):
    def __init__(self, tag):
        super(RemoveOp, self).__init__(None, tag)

    def _json_data(self, json_dict):
        super(RemoveOp, self)._json_data(json_dict)
        del json_dict['op']


class TagNode(StampedData):
    def __init__(self, stamp, tag, op, remove=False):
        super(TagNode, self).__init__(stamp, tag=tag, op=op, before=[], after=[], remove=remove)

    def width(self):
        return sum([x.width() for x in self.before + self.after]) + (1 if self.op is not None else 0)

class TaggedOperation(StampedData):
    def __init__(self, stamp, tag, op):
        super(TaggedOperation, self).__init__(stamp, tag=tag, op=op)

class TaggedIOOperation(StampedData):
    def __init__(self, stamp, tag, op):
        super(TaggedIOOperation, self).__init__(stamp, tag=tag, 
                                                 inputs=[p for p in op.args_paths.values() if type(p) is Path],
                                                 outputs=op._root_set.values())


class OperationException(Exception):
    pass

class Constraint(object):
    def __init__(self, lower, upper, expr):
        self.lower = lower
        self.upper = upper
        self.expr  = expr

    @property
    def free_symbols(self):
        out = set()
        if hasattr(self.lower, 'free_symbols'):
            out.update(self.lower.free_symbols)
        if hasattr(self.upper, 'free_symbols'):
            out.update(self.upper.free_symbols) 
        if hasattr(self.expr, 'free_symbols'):
            out.update(self.expr.free_symbols)
        return out

    def __str__(self):
        s_low  = str(self.lower) if not hasattr(self.lower, 'latex') else self.lower.latex()
        s_upp  = str(self.upper) if not hasattr(self.upper, 'latex') else self.upper.latex()
        s_expr = str(self.expr)  if not hasattr(self.expr, 'latex')  else self.expr.latex()
        return '{}\n  >= {}\n  <= {}'.format(s_expr, s_low, s_upp)

    def __eq__(self, other):
        if isinstance(other, Constraint):
            return self.lower == other.lower and self.upper == other.upper and self.expr == other.expr
        return False


class ArticulationModel(object):
    def __init__(self):
        self.data_tree         = DataTree()
        self.operation_history = History()
        self.timeline_tags     = {}
        self.constraints       = {}
        self.constraint_symbol_map = {}
        self._touched_set      = set()
        self._touched_stamp    = 0


    @profile
    def save_to_file(self, f, subpath=Path('')):
        self.clean_structure()
        if len(subpath) != 0:
            json.dump([{'op' : top.op, 'tag' : top.tag} for top in self.get_history_of(subpath)], f)
        else:
            json.dump([{'op' : chunk.operation, 'tag' : t} for (_, t), chunk in zip(sorted([(s, t) for t, s in self.timeline_tags.items()]), self.operation_history.chunk_history)], f)


    @profile
    def load_from_file(self, f): #, clear_model=False):
        self.clean_structure()
        instructions = json.load(f)
        for x, d in enumerate(instructions):
            if 'tag' not in d:
                raise Exception('Loading of model from file failed: Entry {} has no tag-field.'.format(x))

            if d['tag'] is None:
                raise Exception('Loading of model from file failed: Entry {} has a NULL tag.'.format(x))

            if 'op' not in d:
                raise Exception('Loading of model from file failed: Tag "{}" has no op-field.'.format(d['tag']))

            if d['op'] is None:
                raise Exception('Loading of model from file failed: Tag "{}" has a NULL operation.'.format(d['tag']))

            self.apply_operation(d['tag'], d['op'])
        self.clean_structure()

    @profile
    @type_check(Tag, Operation)
    def apply_operation(self, tag, op):
        time = self.timeline_tags[tag] if tag in self.timeline_tags else self.operation_history.get_time_stamp()
        self.clean_structure(time)

        if tag in self.timeline_tags:
            chunk = self.operation_history.get_chunk(self.timeline_tags[tag])
            if chunk is None:
                raise Exception('History returned no chunk for tag "{}" associated with timestamp {}. This should not happen.'.format(tag, self.timeline_tags[tag]))
            chunk.operation.revoke(self)
            try:
                self.operation_history.replace_chunk(chunk, Chunk(time, op))
                op.apply(self)
            except Exception as e:
                raise OperationException('Failed to apply operation "{}" tagged "{}" at time "{}". Error:\n  {}'.format(op.name, tag, time, e))
        else:
            self.timeline_tags[tag] = time
            try:
                self.operation_history.insert_chunk(Chunk(time, op))
                op.apply(self)
            except Exception as e:
                raise OperationException('Failed to apply operation "{}" tagged "{}" at time "{}". Traceback:\n  {}\nError: \n{}'.format(op.name, tag, time, traceback.print_exc(), e))


    @profile
    @type_check(Tag, Tag, Operation)
    def apply_operation_before(self, tag, before_tag, op):
        if before_tag not in self.timeline_tags:
            raise Exception('History does not contain a timestamp for tag "{}"'.format(before_tag))
        if tag in self.timeline_tags:
            raise Exception('Inserting operations before others can only be done for new operations. Tag "{}" is already refering to an operation.'.format(tag))

        time = self.operation_history.get_time_stamp(before=self.timeline_tags[before_tag])
        self.clean_structure(time)
        self.timeline_tags[tag] = time
        try:
            self.operation_history.insert_chunk(Chunk(time, op))
            op.apply(self)
        except Exception as e:
            raise OperationException('Failed to apply operation "{}" tagged "{}" at time "{}". Traceback:\n  {}\nError: \n{}'.format(op.name, tag, time, traceback.print_exc(), e))


    @profile
    @type_check(Tag, Tag, Operation)
    def apply_operation_after(self, tag, after_tag, op):
        if after_tag not in self.timeline_tags:
            raise Exception('History does not contain a timestamp for tag "{}"'.format(after_tag))
        if tag in self.timeline_tags:    
            raise Exception('Inserting operations after others can only be done for new operations. Tag "{}" is already refering to an operation.'.format(tag))

        time = self.operation_history.get_time_stamp(after=self.timeline_tags[after_tag])
        self.clean_structure(time)
        self.timeline_tags[tag] = time
        try:
            self.operation_history.insert_chunk(Chunk(time, op))
            op.apply(self)
        except Exception as e:
            raise OperationException('Failed to apply operation "{}" tagged "{}" at time "{}". Traceback:\n  {}\nError: \n{}'.format(op.name, tag, time, traceback.print_exc(), e))


    @profile
    @type_check(Tag)
    def remove_operation(self, tag):
        if tag not in self.timeline_tags:
            raise Exception('Tag "{}" not found in time line.'.format(tag))
        chunk = self.operation_history.get_chunk(self.timeline_tags[tag])
        if chunk is None:
            raise Exception('History returned no chunk for tag "{}" associated with timestamp {}. This should not happen.'.format(tag, self.timeline_tags[tag]))
        chunk.operation.revoke(self)
        self.operation_history.remove_chunk(chunk)
        del self.timeline_tags[tag]


    @profile
    @type_check(Tag)
    def get_operation(self, tag):
        if tag not in self.timeline_tags:
            raise Exception('Tag "{}" not found in time line.'.format(tag))
        return self.operation_history.get_chunk(self.timeline_tags[tag]).operation

    @profile
    def clean_structure(self, until=2e9):
        while len(self.operation_history.dirty_chunks) > 0 and self.operation_history.dirty_chunks[0].stamp <= until:
            chunk = self.operation_history.dirty_chunks[0]
            if self._touched_stamp > chunk.stamp:
                self._touched_set = set()
            chunk.operation.apply(self, self._touched_set)
            self._touched_set  |= chunk.modifications
            self._touched_stamp = chunk.stamp
            self.operation_history.flag_clean(chunk)
            self.operation_history.flag_dirty(*chunk.dependents)            

    @profile
    def has_data(self, key):
        if type(key) == str:
            key = Path(key)
        return key in self.data_tree

    @profile
    def get_data(self, key):
        if type(key) == str:
            key = Path(key)
        return self.data_tree[key]

    @profile
    def set_data(self, key, value):
        if type(key) == str:
            key = Path(key)
        self.data_tree[key] = value

    @profile
    def remove_data(self, key):
        if type(key) == str:
            key = Path(key)
        self.data_tree.remove_data(key)

    @profile
    def add_constraint(self, key, constraint):
        if key in self.constraints:
            c = self.constraints[key]
            for s in c.expr.free_symbols:
                self.constraint_symbol_map[s].remove(key)    
        self.constraints[key] = constraint
        for s in constraint.expr.free_symbols:
            if s not in self.constraint_symbol_map:
                self.constraint_symbol_map[s] = set()
            self.constraint_symbol_map[s].add(key)

    def has_constraint(self, key):
        return key in self.constraints

    def get_constraint(self, key):
        return self.constraints[key]

    def list_constraints(self):
        return sorted(self.constraints.keys())

    def remove_constraint(self, key):
        c = self.constraints[key]
        for s in c.expr.free_symbols:
            self.constraint_symbol_map[s].remove(key)
        del self.constraints[key]

    @profile
    def get_constraints_by_symbols(self, symbol_set):
        out = {}
        for s in symbol_set:
            if s in self.constraint_symbol_map:
                out.update({k: self.constraints[k] for k in self.constraint_symbol_map[s]})
        return out

    def get_constant_constraints(self, symbol):
        out = {}
        if symbol in self.constraint_symbol_map:
            for k in self.constraint_symbol_map[symbol]:
                c = self.constraints[k]
                if not is_symbolic(c.lower) and not is_symbolic(c.upper) and c.expr == symbol:
                    out[k] = c
        return out

    def has_tag(self, tag):
        return tag in self.timeline_tags

    def get_tag_stamp(self, tag):
        if tag not in self.timeline_tags:
            raise Exception('Tag "{}" is unknown.'.format(tag))
        return self.timeline_tags[tag]

    def get_tagged_operation(self, tag):
        stamp = self.get_tag_stamp(tag)
        return self.operation_history.get_chunk(tag).operation

    def get_history_of(self, *paths):
        inv_tags = {s: t for t, s in self.timeline_tags.items()}
        return [TaggedOperation(c.stamp, inv_tags[c.stamp], c.operation) for c in self.operation_history.get_history_of(*paths)]

    def get_data_chain(self):
        inv_tags = {s: t for t, s in self.timeline_tags.items()}
        return [TaggedIOOperation(c.stamp, inv_tags[c.stamp], c.operation) for c in self.operation_history]

    def str_op_history(self):
        return '\n'.join(['{:>9.4f}: {}'.format(s, t) for s, t in sorted([(s, t) for t, s in self.timeline_tags.items()])])

    @profile
    def merge_operations_timeline(self, timeline, deleted_tags=[]):
        if not isinstance(timeline, Timeline):
            raise Exception('Merging only works with timeline type.')

        for d in deleted_tags:
            timeline.add(StampedData(self.timeline_tags[d], tag=tag, op=None))

        for tagged_op in timeline:
            if tagged_op.op is None:
                self.remove_operation(tagged_op.tag)
            else:
                self.clean_structure(tagged_op.stamp)
                if tagged_op.tag in self.timeline_tags:
                    tl_stamp = self.timeline_tags[tagged_op.tag]
                    if tl_stamp != tagged_op.stamp:
                        raise Exception('Conflicting time stamps found during merge. Stamp for tag "{}" is {} in timeline, but "{}" in the merging timeline.'.format(tagged_op.tag, tl_stamp, tagged_op.stamp))

                    chunk = self.operation_history.get_chunk(tagged_op.stamp)
                    chunk.operation.revoke(self)
                    self.operation_history.replace_chunk(chunk, Chunk(tagged_op.stamp, tagged_op.op))
                else:
                    self.timeline_tags[tagged_op.tag] = tagged_op.stamp
                    self.operation_history.insert_chunk(Chunk(tagged_op.stamp, tagged_op.op))

                tagged_op.op.apply(self)

    @profile
    def apply_instructions(self, instructions):
        mods     = Timeline()
        nodes    = {}
        new_ops  = []
        removals = set()

        # Construct and operations tree
        for instr in instructions:
            if type(instr) == ApplyAt:
                if instr.tag in self.timeline_tags and instr.tag not in removals:
                    if instr.tag in nodes:
                        node = nodes[instr.tag]
                        node.op = instr.op
                    else:
                        node = TagNode(self.timeline_tags[instr.tag], instr.tag, instr.op)
                        mods.add(node)
                else:
                    removals.discard(instr.tag)
                    node = TagNode(0, instr.tag, instr.op)
                    new_ops.append(node)
                
            elif type(instr) == ApplyBefore or type(instr) == ApplyAfter:
                if instr.tag in self.timeline_tags:
                    raise Exception('Relative insertion of operations is only possible with new tags. Offending tag: "{}"'.format(instr.tag))
                
                if instr.tag in nodes and instr.tag not in removals:
                    raise Exception('Relative insertion of operations is only possible with new tags. Tag "{}" was already used in the given batch.'.format(instr.tag))    

                if type(instr) == ApplyAfter and instr.ref_tag in removals:
                    raise Exception('Inserting of operation tagged "{}" after "{}" is not possible, because the referenced operation is removed.'.format(instr.tag, instr.ref_tag)) 

                if instr.ref_tag in nodes:
                    ref_node = nodes[instr.ref_tag]
                else:
                    if instr.ref_tag in self.timeline_tags:
                        ref_node = TagNode(self.timeline_tags[ref_node], instr.ref_tag, None)
                    else:
                        raise Exception('Reference tag "{}" can not be found in timeline.'.format(instr.ref_tag))

                node = TagNode(0, instr.tag, instr.op)
                if type(instr) == ApplyBefore:
                    ref_node.before.append(node)
                else:
                    ref_node.after.append(node)
            elif type(instr) == RemoveOp:
                if instr.tag in nodes:
                    node = nodes[instr.tag]
                    node.remove = True
                elif instr.tag in self.timeline_tags:
                    node = TagNode(self.timeline_tags[instr.tag], instr.tag, None, remove=True)
                    mods.add(node)
                else:
                    raise Exception('Can not remove operation tagged "{}" because it does not exist in either the instructions batch, nor the timeline.'.format(instr.tag))
                removals.add(instr.tag)
            else:
                raise Exception('Type of instruction is not recognized. Type: "{}"'.format(type(instr)))
            
            nodes[instr.tag] = node

        for n in list(mods) + new_ops:
            self._apply_tag_node(n)

    @profile
    def _apply_tag_node(self, node):
        if node.remove:
            for b in node.before:
                self._apply_tag_node_before(b, node.tag)
            self.remove_operation(node.tag)
        else:
            if node.tag in self.timeline_tags:
                for b in node.before:
                    self._apply_tag_node_before(b, node.tag)

                if node.op is not None:
                    self.apply_operation(node.tag, node.op)

                for a in node.after:
                    self._apply_tag_node_after(a, node.tag)
            else: # New operation
                for b in node.before:
                    self._apply_tag_node(b)

                self.apply_operation(node.tag, node.op)

                # Performance gain
                for a in reversed(node.after):
                    self._apply_tag_node(a)

    @profile
    def _apply_tag_node_before(self, node, before_tag):
        if node.remove:
            raise Exception('"Before"-semantic is non-sensical for remove operations. Node-tag: "{}"'.format(node.tag))

        # This can all be accelerated quite a bit
        # chunk_ub, ub_pos = self.operation_history.get_chunk_pos(self.timeline_tags[before_tag])
        # lb_chunk = None
        # if ub_pos > 0:
        #     lb_chunk = self.operation_history.get_chunk_by_index(ub_pos - 1)

        # n_ops = len(node)
        
        self.apply_operation_before(node.tag, before_tag, node.op)
        for b in node.before:
            self._apply_tag_node_before(b, node.tag)

        for a in node.after:
            self._apply_tag_node_after(a, node.tag)

    @profile
    def _apply_tag_node_after(self, node, after_tag):
        if node.remove:
            raise Exception('"After"-semantic is non-sensical for remove operations. Node-tag: "{}"'.format(node.tag))

        _, ub_pos = self.operation_history.get_chunk_pos(self.timeline_tags[after_tag])
        if ub_pos == len(self.operation_history) - 1:
            for b in node.before:
                self._apply_tag_node(b)
            
            self.apply_operation(node.tag, node.op)

            for a in reversed(node.after):
                self._apply_tag_node(a)
        else:
            # This can all be accelerated quite a bit
            self.apply_operation_after(node.tag, after_tag, node.op)
            for b in node.before:
                self._apply_tag_node_before(b, node.tag)

            for a in node.after:
                self._apply_tag_node_after(a, node.tag)


    def __eq__(self, other):
        if isinstance(other, ArticulationModel):
            return self.operation_history == other.operation_history and self.data_tree == other.data_tree and self.timeline_tags == other.timeline_tags and self.constraints == other.constraints 
        return False
