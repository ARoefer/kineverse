import rospy

import kineverse.network.names as stdn

from multiprocessing import RLock

from kineverse.model.history          import Timeline
from kineverse.model.event_model      import EventModel
from kineverse.model.kinematic_model  import TaggedOperation, ApplyAt, ApplyBefore, ApplyAfter, RemoveOp
from kineverse.model.paths            import Path, PathSet
from kineverse.network.ros_conversion import decode_op_msg, encode_operation_instruction
from kineverse.time_wrapper           import Time

from kineverse.msg import Operation as OperationMsg
from kineverse.msg import OperationCall as OperationCallMsg
from kineverse.msg import OperationsUpdate as OperationsUpdateMsg

from kineverse.srv import GetHistory      as GetHistorySrv
from kineverse.srv import ApplyOperations as ApplyOperationsSrv


class OperationsClient_NoROS(object):
    def __init__(self, model_type, buffered, *args):
        if model_type is not None and not issubclass(model_type, EventModel):
            raise Exception('Model type must be a subclass of EventModel. Type "{}" is not.'.format(model_type))

        self.km = model_type(*args) if model_type is not None else EventModel()

        self.lock          = RLock()

        self.buffered = buffered

        self._last_update  = Time(0)
        self.tracked_paths = PathSet()

        self._indicator_paths       = PathSet()
        self._tracker_indicator_map = {}
        self._operations_batch      = []

    def kill(self):
        pass

    def has_data(self, path):
        if type(path) == str:
            path = Path(path)

        with self.lock:
            if path not in self.tracked_paths: 
                self._start_tracking(path)
            return self.km.has_data(path)


    def get_data(self, data):
        if type(path) == str:
            path = Path(path)

        with self.lock:
            if path not in self.tracked_paths: 
                self._start_tracking(path)
            return self.km.get_data(path)


    def _update_operations(self, msg):
        tl = Timeline()
        temp_tl = Timeline([TaggedOperation(s, t, op_msg) for s, t, op_msg in zip(msg.stamps, msg.tags, msg.operations)])
        for to in reversed(temp_tl): # Iterate backwards to deal with newly arising dependencies
            if not self.km.has_tag(to.tag): # Operation is unknown
   
                to.op = decode_op_msg(to.op)
                # If operation modifies a path in our indicator set, add it
                if max([p in self._indicator_paths for p in to.op._root_set.values()]):
                    tl.add(to)
                    # Monitor paths this operation depends on
                    self._indicator_paths.update({p for p in to.args_paths.values() if type(p) is Path})
            else: # Update operations we already know to care about
                to.op = decode_op_msg(to.op)
                tl.add(to)
                # Monitor paths this operation depends on. TODO: SEEMS SUB-OPTIMAL
                self._indicator_paths.update({p for p in to.args_paths.values() if type(p) is Path})

        relevant_deletions = [x for x in msg.deleted if self.km.has_tag(x)]
        self.km.merge_operations_timeline(tl)


    def cb_operations_update(self, msg):
        with self.lock:
            if msg.stamp < self._last_update:
                return

            self._last_update = msg.stamp

            self._update_operations(msg)

            self.km.clean_structure()
            self.km.dispatch_events()


    def _start_tracking(self, path):
        self.tracked_paths.add(path)
        self._update_operations(self.srv_get_history([str(path)]).history)
        self.km.dispatch_events()

    def _check_tracked(self, operation):
        mod_paths = set(operation._root_set.values())
        if mod_paths.intersection(self.tracked_paths) < len(mod_paths):
            for p in mod_paths:
                if p not in self.tracked_paths:
                    self._start_tracking(p)

    def _add_instruction(self, instr):
        with self.lock:
            if type(instr) is not RemoveOp:
                self._check_tracked(instr.op)
            self._operations_batch.append(instr)

            if not self.buffered:
                self.apply_changes()


    def apply_operation(self, op, tag):
        self._add_instruction(ApplyAt(op, tag))

    def apply_operation_before(self, op, tag, before):
        self._add_instruction(ApplyBefore(op, tag, before))

    def apply_operation_after(self, op, tag, after):
        self._add_instruction(ApplyAfter(op, tag, after))

    def remove_operation(self, tag):
        if not self.km.has_tag(tag):
            raise Exception('Can not remove operation "{}" as it is not found in local model.'.format(tag))
        self._add_instruction(RemoveOp(tag))

    def apply_changes(self):
        raise NotImplementedError

    def register_on_model_changed(self, path, callback):
        self.km.register_on_model_changed(path, callback)
        if path not in self.tracked_paths: 
            self._start_tracking(path)

    def deregister_on_model_changed(self, callback):
        self.km.deregister_on_model_changed(callback)    

    def register_on_constraints_changed(self, symbols, callback):
        self.km.register_on_constraints_changed(symbols, callback)
        self.tracked_symbols |= symbols

    def deregister_on_constraints_changed(self, callback):
        self.km.deregister_on_constraints_changed(callback)


class OperationsClient(OperationsClient_NoROS):
    def __init__(self, model_type, buffered, *args):
        super(OperationsClient, self).__init__(model_type, buffered, *args)

        rospy.wait_for_service(stdn.srv_apply_op)
        self.srv_apply_operations = rospy.ServiceProxy(stdn.srv_apply_op, ApplyOperationsSrv)
        
        rospy.wait_for_service(stdn.srv_get_history)
        self.srv_get_history      = rospy.ServiceProxy(stdn.srv_get_history, GetHistorySrv)

        self.sub_ops = rospy.Subscriber(stdn.topic_update_ops, OperationsUpdateMsg, self.cb_operations_update, queue_size=1)

    def apply_changes(self):
        with self.lock:
            req = [encode_operation_instruction(x) for x in self._operations_batch]
            #print(req)
            res = self.srv_apply_operations(req)
            self._operations_batch = []
            if not res.success:
                raise Exception(res.error_msg)

    def kill(self):
        with self.lock:
            self.sub_ops.unregister()