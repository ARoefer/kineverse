from multiprocessing import RLock

from kineverse.model.history          import Timeline
from kineverse.model.event_model      import EventModel
from kineverse.model.articulation_model  import TaggedOperation, \
                                                ApplyAt, \
                                                ApplyBefore, \
                                                ApplyAfter, \
                                                RemoveOp, \
                                                Tag
from kineverse.model.paths            import Path, PathSet
from kineverse.operations.operation   import Operation
from kineverse.network.ros_conversion import decode_op_msg, encode_operation_instruction
from kineverse.time_wrapper           import Time

from kineverse_msgs.msg import Operation as OperationMsg
from kineverse_msgs.msg import OperationCall as OperationCallMsg
from kineverse_msgs.msg import OperationsUpdate as OperationsUpdateMsg

from kineverse_msgs.srv import GetHistory      as GetHistorySrv
from kineverse_msgs.srv import ApplyOperations as ApplyOperationsSrv

class OperationsClient(object):
    def __init__(self, model_type, *model_args):
        self.km = model_type(*model_args) if model_type is not None else EventModel
        self.tracked_paths = set()
        self.tracked_tags  = set()
        self._last_update  = Time(0)

        self.srv_get_history     = NotImplemented
        self.srv_apply_operation = NotImplemented

        self.lock = RLock()

    def has_data(self, path):
        if not isinstace(path, Path):
            path = Path(path)

        with self.lock:
            if path not in self.tracked_paths:
                self._start_tracking(path)

            return self.km.has_data(path)


    def get_data(self, path):
        if not isinstace(path, Path):
            path = Path(path)

        with self.lock:
            self._start_tracking(path)

            return self.km.get_data(path)


    def _start_tracking(self, path):
        if not path in self.tracked_paths:
            self.tracked_paths.add(path)
            if not self.km.has_data(path):
                self.process_operations_update(self.srv_get_history([path]).history)

    @type_check(Tag, Operation)
    def apply_operation(self, tag, op):
        self.tracked_tags.add(tag)
        self.srv_apply_operation(encode_operation_instruction(ApplyAt(op, tag)))


    @type_check(Tag, Tag, Operation)
    def apply_operation_before(self, tag, before_tag, op):
        self.tracked_tags.add(tag)

    @type_check(Tag, Tag, Operation)
    def apply_operation_before(self, tag, before_tag, op):
        self.tracked_tags.add(tag)

    @type_check(Tag)
    def remove_operation(self, tag):
        self.tracked_tags.discard(tag)

    @type_check(OperationsUpdateMsg)
    def process_operations_update(self, update_msg):
        if update_msg.stamp < self._last_update:
            return

        stamped_ops = [StampedData(stamp, tag=tag, json_op=op)
                       for tag, stamp, op in zip(update_msg.tags, 
                                                 update_msg.stamps, 
                                                 update_msg.operations)]

        for sop in sorted(stamped_ops):
            sop.op = decode_op_msg(sop.op)
            if sop.tag in self.tracked_tags or len(self.tracked_paths.intersection(sop.output_path_assignments.values())) > 0:
                self.km._place_operation(sop.stamp, sop.tag, sop.op)

        self.km.clean_structure()
        self.km.dispatch_events()


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
