import rospy

from kineverse.model.tardis_wrapper import TARDIS
from kineverse.model.history        import Timeline, StampedData
from kineverse.utils                import import_class
from kineverse.yaml_wrapper         import yaml

from kineverse.msg import Constraint    as ConstraintMsg
from kineverse.msg import OperationCall as OperationCallMsg 
from kineverse.msg import ModelUpdate   as ModelUpdateMsg

from kineverse.srv import GetModel               as GetModelSrv
from kineverse.srv import GetModelResponse       as GetModelResponseMsg 
from kineverse.srv import GetConstraints         as GetConstraintsSrv
from kineverse.srv import GetConstraintsResponse as GetConstraintsResponseMsg 

operations_registry = {}


def decode_op_msg(msg):
    if op_msg.operation_type not in operations_registry:
        if op_msg.operation_type[:8] == '<class \'' and op_msg.operation_type[-2:] == '\'>':
            operations_registry[op_msg.operation_type] = import_class(op_msg.operation_type[8:-2])
        else:
            raise Exception('Operation type "{}" does not match the pattern "<class \'TYPE\'>"'.format(op_msg.operation_type))
    return operations_registry[op_msg.operation_type](*[yaml.full_load(x) for x in msg.operations])    


class TagNode(StampedData):
    def __init__(self, stamp, tag, op):
        super(TagNode, self).__init__(stamp, tag=tag, op=op, before=[], after=[])


class ModelServer(object):
    def __init__(self, model_type, *args):
        self.km = TARDIS(model_type, *args)

        self.pub_model = rospy.Publisher('/km', ModelUpdateMsg, queue_size=1)
        self.pub_ops   = rospy.Publisher('/ko', OperationCallMsg, queue_size=1)

        self.services  = [
            rospy.Service('get_model', GetModelSrv, self.srv_get_model),
            rospy.Service('get_constraints', GetConstraintsSrv, self.srv_get_constraints)]

        self._changed_set = set()
        self._changed_constraints = set()

    def _apply_tag_node(self, node):
        if node.op == False: # Removal
            for b in node.before:
                self._apply_tag_node_before(b, node.tag)
            op = self.km.get_operation(node.tag)
            self._changed_set.update(op._root_set.values())
            self._changed_constraints.update(op._constraint_memento.keys())
            self.km.remove_operation(node.tag)
        else:
            if node.op is not None:
                if not self.km.has_tag(node.tag): # The operation is new: Execute linearely
                    for b in node.before:
                        self._apply_tag_node(b)
                    
                    op = decode_op_msg(node.op)
                    self.km.apply_operation(op, node.tag)
                    self._changed_set.update(op._root_set.values())
                    self._changed_constraints.update(op._constraint_memento.keys())

                    for a in reversed(node.after):
                        self._apply_tag_node(a)
                else: # Operation is old
                    for b in node.before:
                        self._apply_tag_node_before(b, node.tag)
                    
                    op = decode_op_msg(node.op)
                    self.km.apply_operation(op, node.tag)
                    self._changed_set.update(op._root_set.values())
                    self._changed_constraints.update(op._constraint_memento.keys())

                    for a in node.after:
                        self._apply_tag_node_after(a, node.tag_node)
            else: # There is no main operation
                for b in node.before:
                        self._apply_tag_node_before(b, node.tag)
                
                for a in node.after:
                    self._apply_tag_node_after(a, node.tag_node)

    def _apply_tag_node_before(self, node, before):
        op = decode_op_msg(node.op)
        self.km.apply_operation_before(op, node.tag, before)
        self._changed_set.update(op._root_set.values())

        for b in node.before:
            self._apply_tag_node_before(b, node.tag)
        for a in node.after:
            self._apply_tag_node_after(a, node.tag)

    def _apply_tag_node_after(self, node, after):
        op = decode_op_msg(node.op)
        self.km.apply_operation_after(op, node.tag, after)
        self._changed_set.update(op._root_set.values())

        for b in node.before:
            self._apply_tag_node_before(b, node.tag)
        for a in node.after:
            self._apply_tag_node_after(a, node.tag)


    def process_operations_msg(self, msg):
        old_ops    = Timeline()
        tag_nodes  = {}
        new_ops    = []
        removals   = set()
        for op in msg.operations:
            if op.call_mode == OperationCallMsg.REMOVE_OP:
                tag_node = TagNode(self.km.get_tag_stamp(op.tag), op.tag, False)
                old_ops.add(tag_node)
                removals.add(op.tag)
            elif op.call_mode == OperationCallMsg.INSERT_AT:
                if op.tag in tag_nodes:
                    if op.tag not in removals:
                        tag_nodes[op.tag].op = op
                    else:
                        tag_node = TagNode(0, op.tag, op)
                        tag_nodes[tag_node.tag] = tag_node
                        new_ops.append(tag_node)
                else:
                    if self.km.has_tag(op) and op.tag not in removals:
                        tag_node = TagNode(self.km.get_tag_stamp(op.tag), op.tag, op_msg)
                        old_ops.add(tag_node)
                    else:
                        tag_node = TagNode(0, op.tag, op_msg)
                        removals.discard(op.tag)
                        new_ops.append(tag_node)

                    tag_nodes[tag_node.tag] = tag_node
            elif op.call_mode == OperationCallMsg.INSERT_BEFORE:
                if self.km.has_tag(op.tag):
                    raise Exception('Insertion before operations is only supported for new operations. Tag "{}" is already taken.'.format(op.tag))

                tag_node = TagNode(0, op.tag, op_msg)
                if op.reference_tag in tag_nodes:
                    ref_node = tag_nodes[op.reference_tag]  
                else:
                    ref_node = TagNode(self.km.get_tag_stamp(op.reference_tag), op.reference_tag, None)
                    tag_nodes[ref_node.tag] = ref_node
                    old_ops.add(ref_node)
                ref_node.before.append(tag_node)
                tag_nodes[tag_node.tag] = tag_node
            elif op.call_mode == OperationCallMsg.INSERT_AFTER:
                if self.km.has_tag(op.tag):
                    raise Exception('Insertion after operations is only supported for new operations. Tag "{}" is already taken.'.format(op.tag))

                tag_node = TagNode(0, op.tag, op_msg)
                if op.reference_tag in tag_nodes:
                    ref_node = tag_nodes[op.reference_tag]
                    if ref_node.op is False:
                        raise Exception('Can not execute operation based on removed operations. Operation "{}" is based on "{}"'.format(tag_node.tag, ref_node.tag))
                else:
                    ref_node = TagNode(self.km.get_tag_stamp(op.reference_tag), op.reference_tag, None)
                    tag_nodes[ref_node.tag] = ref_node
                    old_ops.add(ref_node)
                ref_node.after.append(tag_node)
                tag_nodes[tag_node.tag] = tag_node

        for node in list(old_ops) + new_ops:
            self._apply_tag_node(node)


        self.km.clean_structure()
        self.km.dispatch_events()

        model_update_msg = ModelUpdateMsg()
        model_update_msg.stamp = rospy.Time.now()
        for p in self._changed_set:
            if self.km.has_data(p):
                model_update_msg.update.paths.append(str(p))
                model_update_msg.update.data.append(yaml.full_dump(self.km.get_data(p)))
            else:
                model_update_msg.deleted_paths.append(str(p))

        for k in self._changed_constraints:
            if self.km.has_constraint(k):
                cm = ConstraintMsg()
                c  = km.get_constraint(k)
                cm.name  = k
                cm.lower = yaml.full_dump(c.lower)
                cm.upper = yaml.full_dump(c.upper)
                cm.expr  = yaml.full_dump(c.expr)
                model_update_msg.update.constraints.append(cm)
            else:
                cm.deleted_constraints.append(k)

        self.pub_model.publish(model_update_msg)


    def srv_get_model(self, req):
        res = GetModelResponseMsg()
        for p in req.paths:
            if self.km.has_data(p):
                res.model.paths.append(p)
                res.model.data.append(yaml.dump(self.km.get_data(p)))

        return res

    def srv_get_constraints(self, req):
        res = GetConstraintsResponseMsg()
        for k, c in self.km.get_constraints_by_symbols({spw.Symbol(s) for s in req.symbols}).items():
            cm = ConstraintMsg()
            cm.name  = k
            cm.lower = yaml.full_dump(c.lower)
            cm.upper = yaml.full_dump(c.upper)
            cm.expr  = yaml.full_dump(c.expr)
            res.constraints.append(cm)
        return res