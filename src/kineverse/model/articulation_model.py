"""
The articulation_model module contains the basic implementation of articulation
models in Kineverse.
The basic model provides data storage and access, query functionality for constraints,
and operational model building which includes merging of models.
"""

import traceback 
import kineverse.gradients.common_math as cm
import kineverse.json_wrapper          as json
import kineverse.model.model_settings  as model_settings

from kineverse.model.data_tree             import DataTree
from kineverse.model.history               import History, Timeline, StampedData, Chunk
from kineverse.model.paths                 import Path
from kineverse.operations.operation        import Operation
# from kineverse.operations.basic_operations import CreateSingleValue
from kineverse.type_sets                   import is_symbolic
from kineverse.json_serializable           import JSONSerializable

Tag = str

class ApplicationInstruction(JSONSerializable):
    """General data type for serializing the application of
    and operation to a model.
    """
    @type_check(Operation, Tag)
    def __init__(self, op, tag):
        self.op  = op
        self.tag = tag

    def _json_data(self, json_dict):
        json_dict['op'] = self.op
        json_dict['tag'] = self.tag


class ApplyAt(ApplicationInstruction):
    """Operation applied at the end of the history, or at
    the same tag.
    """
    def __init__(self, op, tag):
        super(ApplyAt, self).__init__(op, tag)

class ApplyBefore(ApplicationInstruction):
    """Operation is applied immediately before a reference tag."""
    def __init__(self, op, tag, before):
        super(ApplyBefore, self).__init__(op, tag)
        self.ref_tag = before

    def _json_data(self, json_dict):
        super(ApplyBefore, self)._json_data(json_dict)
        json_dict['before'] = self.ref_tag


class ApplyAfter(ApplicationInstruction):
    """Operation is applied immediately after a reference tag."""
    def __init__(self, op, tag, after):
        super(ApplyAfter, self).__init__(op, tag)
        self.ref_tag = after

    def _json_data(self, json_dict):
        super(ApplyAfter, self)._json_data(json_dict)
        json_dict['after'] = self.ref_tag


class RemoveOp(ApplicationInstruction):
    """Operation with given tag is removed."""
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
                                                 inputs=[p for p in op._exec_args.values() if type(p) is Path],
                                                 outputs=op.output_path_assignments.values())


class OperationException(Exception):
    """Exception thrown when something goes wrong with an operation."""
    pass

class Constraint(object):
    """Kineverse constraint consisting of a lower bound, an upper bound,
    and a constrained expression."""
    def __init__(self, lower, upper, expr):
        self.lower = lower
        self.upper = upper
        self.expr  = expr

    @property
    def free_symbols(self):
        """Returns the union of free symbols of all three constraint components."""
        return cm.free_symbols(self.lower) | cm.free_symbols(self.upper) | cm.free_symbols(self.expr)

    def __str__(self):
        s_low  = str(self.lower) if not hasattr(self.lower, 'latex') else self.lower.latex()
        s_upp  = str(self.upper) if not hasattr(self.upper, 'latex') else self.upper.latex()
        s_expr = str(self.expr)  if not hasattr(self.expr, 'latex')  else self.expr.latex()
        return '{}\n  >= {}\n  <= {}'.format(s_expr, s_low, s_upp)

    def __eq__(self, other):
        """Checks equivalence to other constraint. This is an iffy test with casadi."""
        if isinstance(other, Constraint):
            return cm.eq_expr(self.lower, other.lower) and \
                   cm.eq_expr(self.upper, other.upper) and \
                   cm.eq_expr(self.expr, other.expr)
        return False


class ArticulationModel(object):
    """Base articulation model implementation."""
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
        """Saves the current model to a JSON file, by serializing
        the history of operations.

        :param f: File object
        :param subpath: Path of the model to serialize. By default the entire model is saved.
        """
        self.clean_structure()
        if len(subpath) != 0:
            json.dump([{'op' : top.op, 'tag' : top.tag} for top in self.get_history_of(subpath)], f)
        else:
            json.dump([{'op' : chunk.operation, 'tag' : t} for (_, t), chunk in zip(sorted([(s, t) for t, s in self.timeline_tags.items()]), self.operation_history.chunk_history)], f)


    @profile
    def load_from_file(self, f): #, clear_model=False):
        """Loads a model form a given file by decoding and applying
        the stored operations. 
        This will merge models, if operations have the same tags.
        """
        self.clean_structure()

        if len(prefix) > 0:
            for x in range(len(prefix) + 1):
                if not self.has_data(prefix[:x]):
                    self.apply_operation('create {}'.format(prefix[:x]), CreateSingleValue(prefix[:x], {}))

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

            if len(prefix) == 0:
                self.apply_operation(d['tag'], d['op'])
            else:
                self.apply_operation(d['tag'], type(d['op'])(*[a if type(a) != Path else prefix + a for a in d['op']._construction_args]))
        self.clean_structure()

    @profile
    @type_check(Tag, Operation)
    def apply_operation(self, tag, op):
        """Apply a single operation to the model. Operation is added at 
        the end, or replaces an operation with the same tag.

        :param tag: Tag to apply the operation with. If tag is already taken, the existing operation is replaced.
        :type  tag: Tag
        :param  op: Operation to apply.
        :type   op: Operation
        """
        time = self.timeline_tags[tag] if tag in self.timeline_tags else self.operation_history.get_time_stamp()
        self.clean_structure(time)

        if tag in self.timeline_tags:
            chunk = self.operation_history.get_chunk(self.timeline_tags[tag])
            if chunk is None:
                raise Exception('History returned no chunk for tag "{}" associated with timestamp {}. This should not happen.'.format(tag, self.timeline_tags[tag]))
            chunk.operation.revoke(self)
            try:
                # Execute the operation to generate full modification set and dependencies
                op.execute(self, time)
                
                # Create a new chunk and try inserting it into the history
                new_chunk = Chunk(time, op)
                self.operation_history.check_can_replace(chunk, new_chunk)

                # Actually apply the generated changes to the model once
                # history insertion worked out
                op.apply_to_model(self, time)
                self.operation_history.replace_chunk(chunk, new_chunk)
                self._touched_set |= new_chunk.modifications
                self._touched_stamp = new_chunk.stamp
            except Exception as e:
                raise OperationException('Failed to apply operation "{}" tagged "{}" at time "{}". Error:\n  {}'.format(type(op), tag, time, e))
        else:
            self.timeline_tags[tag] = time
            try:
                # Execute the operation to generate full modification set and dependencies
                op.execute(self, time)
                
                # Create a new chunk and try inserting it into the history
                new_chunk = Chunk(time, op)
                # op.apply(self, self._touched_set)
                self.operation_history.check_can_insert(new_chunk)

                # Actually apply the generated changes to the model once
                # history insertion worked out
                op.apply_to_model(self, time)
                self.operation_history.insert_chunk(new_chunk)
                self._touched_set |= new_chunk.modifications
                self._touched_stamp = new_chunk.stamp
            except Exception as e:
                raise OperationException('Failed to apply operation "{}" tagged "{}" at time "{}". Traceback:\n  {}\nError: \n{}'.format(type(op), tag, time, traceback.print_exc(), e))


    @profile
    @type_check(Tag, Tag, Operation)
    def apply_operation_before(self, tag, before_tag, op):
        """Apply a single operation to the model. Operation is added immediately
        before the reference tag. An exception is thrown if the given tag already exists.

        :param tag: Tag to apply the operation with. Exception is thrown if the tag already exists.
        :type  tag: Tag
        :param before_tag: Tag before which the operation is applied. Exception is thrown if this tag does not exist.
        :type  before_tag: Tag
        :param  op: Operation to apply.
        :type   op: Operation
        """
        if before_tag not in self.timeline_tags:
            raise Exception('History does not contain a timestamp for tag "{}"'.format(before_tag))
        if tag in self.timeline_tags:
            raise Exception('Inserting operations before others can only be done for new operations. Tag "{}" is already refering to an operation.'.format(tag))

        time = self.operation_history.get_time_stamp(before=self.timeline_tags[before_tag])
        self.clean_structure(time)
        self.timeline_tags[tag] = time
        try:
            # Execute the operation to generate full modification set and dependencies
            op.execute(self, time)

            new_chunk = Chunk(time, op)
            self.operation_history.check_can_insert(new_chunk)
            # op.apply(self, self._touched_set, time)

            op.apply_to_model(self, time)
            self.operation_history.insert_chunk(new_chunk)
            self._touched_set |= new_chunk.modifications
            self._touched_stamp = new_chunk.stamp
        except Exception as e:
            raise OperationException('Failed to apply operation "{}" tagged "{}" at time "{}". Traceback:\n  {}\nError: \n{}'.format(type(op), tag, time, traceback.print_exc(), e))


    @profile
    @type_check(Tag, Tag, Operation)
    def apply_operation_after(self, tag, after_tag, op):
        """Apply a single operation to the model. Operation is added immediately
        after the reference tag. An exception is thrown if the given tag already exists.

        :param tag: Tag to apply the operation with. Exception is thrown if the tag already exists.
        :type  tag: Tag
        :param after_tag: Tag after which the operation is applied. Exception is thrown if this tag does not exist.
        :type  after_tag: Tag
        :param  op: Operation to apply.
        :type   op: Operation
        """
        if after_tag not in self.timeline_tags:
            raise Exception('History does not contain a timestamp for tag "{}"'.format(after_tag))
        if tag in self.timeline_tags:    
            raise Exception('Inserting operations after others can only be done for new operations. Tag "{}" is already refering to an operation.'.format(tag))

        time = self.operation_history.get_time_stamp(after=self.timeline_tags[after_tag])
        self.clean_structure(time)
        self.timeline_tags[tag] = time
        try:
            op.execute(self, time)

            new_chunk = Chunk(time, op)
            self.operation_history.check_can_insert(new_chunk)
            # op.apply(self, self._touched_set, time)
            
            op.apply_to_model(self, time)
            self.operation_history.insert_chunk(new_chunk)
            self._touched_set |= new_chunk.modifications
            self._touched_stamp = new_chunk.stamp
        except Exception as e:
            raise OperationException('Failed to apply operation "{}" tagged "{}" at time "{}". Traceback:\n  {}\nError: \n{}'.format(type(op), tag, time, traceback.print_exc(), e))


    @profile
    @type_check(Tag)
    def remove_operation(self, tag):
        """Removes the operation with the given tag.
        An exception is raise, if the tag is not defined, or removing the operation would break the structure of the history.
        """
        if tag not in self.timeline_tags:
            raise Exception('Tag "{}" not found in time line.'.format(tag))
        chunk = self.operation_history.get_chunk(self.timeline_tags[tag])
        if chunk is None:
            raise Exception('History returned no chunk for tag "{}" associated with timestamp {}. This should not happen.'.format(tag, self.timeline_tags[tag]))
        self.operation_history.check_can_remove(chunk)
        chunk.operation.revoke(self)
        self.operation_history.remove_chunk(chunk)
        self._touched_set |= chunk.modifications
        self._touched_stamp = chunk.stamp if not model_settings.BRUTE_MODE else 0
        del self.timeline_tags[tag]


    @profile
    @type_check(Tag)
    def get_operation(self, tag):
        """Returns the operation for a given tag. Raises an exception if the operation
        does not exist.
        """
        if tag not in self.timeline_tags:
            raise Exception('Tag "{}" not found in time line.'.format(tag))
        return self.operation_history.get_chunk(self.timeline_tags[tag]).operation

    @profile
    def clean_structure(self, until=2e9):
        """Executes all 'dirty' operations until a given time stamp.
        This rebuilds the model after modifications to the history.
        NOTE: The model should be cleaned fully, BEFORE new modifications are made, 
        DO NOT mix modifications and model cleaning.
        """
        # NOTE: Fuck the complicated update mechanism. It causes nothing but problems and is mostly irrelevant to current real-world applications
        if model_settings.BRUTE_MODE:
            # Implies that the model needs to be rebuilt.
            if len(self.operation_history.chunk_history) > 0 and self._touched_stamp < self.operation_history.chunk_history[-1].stamp:
                self.data_tree.clear()

                for chunk in self.operation_history.chunk_history:
                    chunk.operation.execute(self, chunk.stamp)
                    chunk.operation.apply_to_model(self, chunk.stamp)
                    self._touched_stamp = chunk.stamp

                self._touched_set = set()
        else:
            while len(self.operation_history.dirty_chunks) > 0 and self.operation_history.dirty_chunks[0].stamp <= until:
                chunk = self.operation_history.dirty_chunks[0]
                if self._touched_stamp > chunk.stamp:
                    self._touched_set = set()
                chunk.operation.execute(self, chunk.stamp)
                chunk.operation.apply_to_model(self, chunk.stamp)
                self._touched_set  |= chunk.modifications
                self._touched_stamp = chunk.stamp
                self.operation_history.flag_clean(chunk)
                self.operation_history.flag_dirty(*chunk.dependents)
            
            # The entire model is cleaned now
            if len(self.operation_history.chunk_history) == 0 or until >= self.operation_history.chunk_history[-1].stamp:
                self._touched_set = set()

    @profile
    def has_data(self, key, stamp=None):
        """Is the given path contained in the model?
        
        :param key: Path to check for.
        :type  key: str, Path
        """
        if type(key) == str:
            key = Path(key)
        if stamp is None or model_settings.BRUTE_MODE:
            return key in self.data_tree
        else:
            e_stamp = self.operation_history.get_earliest_stamp(key)
            return e_stamp is not None and e_stamp <= stamp

    @profile
    def get_data(self, key, time_stamp=None):
        """Returns the data identified by the given path.
        
        :param key: Path to check for.
        :type  key: str, Path
        """
        if type(key) == str or type(key) == unicode:
            key = Path(key)
        if time_stamp is None or model_settings.BRUTE_MODE:
            return self.data_tree[key]
        else:
            # Fetch the chunk from the history that modifies this path closest after the given stamp.
            # Then extract the value from the chunk's memo.
            if key in self._touched_set and time_stamp >= self._touched_stamp:
                    return self.data_tree[key]
            
            closest_mod_chunk = self.operation_history.get_ceil_modifying_chunk(key, time_stamp)
            if closest_mod_chunk is None:
                return self.data_tree[key]
            return closest_mod_chunk.operation.get_from_memento(key)

    @profile
    def set_data(self, key, value):
        """Stores data under the given path.
        Note: Except for the last bit, the entire path must already exist.
        
        :param key: Path to insert at.
        :type  key: str, Path
        :param value: Data to insert.
        """
        if type(key) == str:
            key = Path(key)
        self.data_tree[key] = value

    @profile
    def remove_data(self, key):
        """Removes the data stored under the given path.
        
        :param key: Path to remove.
        :type  key: str, Path
        """
        if type(key) == str:
            key = Path(key)
        self.data_tree.remove_data(key)

    @profile
    def add_constraint(self, key, constraint):
        """Adds a constraint or replaces one with the same identifier.

        :param key: Identifier of the constraint.
        :type  key: str
        :param constraint: Constraint to add.
        :type  constraint: Constraint
        """
        if key in self.constraints:
            c = self.constraints[key]
            for s in cm.free_symbols(c.expr):
                self.constraint_symbol_map[s].remove(key)    
        self.constraints[key] = constraint
        for s in cm.free_symbols(constraint.expr):
            if s not in self.constraint_symbol_map:
                self.constraint_symbol_map[s] = set()
            self.constraint_symbol_map[s].add(key)

    def has_constraint(self, key):
        """Does a constraint with the given identifier exist?"""
        return key in self.constraints

    def get_constraint(self, key):
        """Return a constraint with a given identifier."""
        return self.constraints[key]

    def list_constraints(self):
        """Return a list of all known constraint identifiers."""
        return sorted(self.constraints.keys())

    def remove_constraint(self, key):
        """Remove a constraint by its identifier."""
        c = self.constraints[key]
        for s in cm.free_symbols(c.expr):
            self.constraint_symbol_map[s].remove(key)
        del self.constraints[key]

    @profile
    def get_constraints_by_symbols(self, symbol_set):
        """Return a dictionary of constraints that affect the symbols from a given set.

        :param symbol_set: Set of symbols out of which a constraint needs to affect at least one.
        :type  symbol_set: {Symbol}
        :return: Dictionary str -> Constraint
        """
        out = {}
        for s in symbol_set:
            if s in self.constraint_symbol_map:
                out.update({k: self.constraints[k] for k in self.constraint_symbol_map[s]})
        return out

    def get_constant_constraints(self, symbol):
        """Return a dictionary of constraints with constant bounds that affect the given symbol.

        :param symbol_set: Set of symbols out of which a constraint needs to affect at least one.
        :type  symbol_set: {Symbol}
        :return: Dictionary str -> Constraint
        """
        out = {}
        if symbol in self.constraint_symbol_map:
            for k in self.constraint_symbol_map[symbol]:
                c = self.constraints[k]
                if not is_symbolic(c.lower) and not is_symbolic(c.upper) and cm.eq_expr(c.expr, symbol):
                    out[k] = c
        return out

    def has_tag(self, tag):
        """Is the given tag defined in the timeline."""
        return tag in self.timeline_tags

    def get_tag_stamp(self, tag):
        """Returns stamp associated with a given tag. Exception raised if tag does not exist."""
        if tag not in self.timeline_tags:
            raise Exception('Tag "{}" is unknown.'.format(tag))
        return self.timeline_tags[tag]

    def get_history_of(self, *paths):
        """Returns the history of a given number of a given set of paths."""
        inv_tags = {s: t for t, s in self.timeline_tags.items()}
        return [TaggedOperation(c.stamp, inv_tags[c.stamp], c.operation) for c in self.operation_history.get_history_of(*paths)]

    def get_data_chain(self):
        """Returns a list of all operations with their full model dependencies and modifications."""
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
        """Applies a list of instructions to the model. 
        Is used to replay actions on a remote model.
        """
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
