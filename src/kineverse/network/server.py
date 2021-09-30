import rospy
import traceback
import datetime

from multiprocessing import RLock

import kineverse.network.names as stdn 

from kineverse.model.paths             import Path, PathDict, collect_paths
from kineverse.model.event_model       import EventModel
from kineverse.model.articulation_model   import ApplyAt, ApplyBefore, ApplyAfter, RemoveOp
from kineverse.model.history           import Timeline, StampedData
from kineverse.utils                   import import_class, res_pkg_path
from kineverse.network.ros_conversion  import json, encode_constraint, decode_op_msg,encode_operation_update, decode_operation_instruction
from kineverse.time_wrapper            import Time
from kineverse.visualization.graph_generator import generate_modifications_graph, generate_dependency_graph, plot_graph

from kineverse_msgs.msg import ModelUpdate      as ModelUpdateMsg
from kineverse_msgs.msg import OperationCall    as OperationCallMsg
from kineverse_msgs.msg import OperationsUpdate as OperationsUpdateMsg

from kineverse_msgs.srv import ApplyOperations         as ApplyOperationsSrv
from kineverse_msgs.srv import ApplyOperationsResponse as ApplyOperationsResponseMsg
from kineverse_msgs.srv import DebugInfo               as DebugInfoSrv
from kineverse_msgs.srv import DebugInfoResponse       as DebugInfoResponseMsg
from kineverse_msgs.srv import GetModel                as GetModelSrv
from kineverse_msgs.srv import GetModelResponse        as GetModelResponseMsg
from kineverse_msgs.srv import GetHistory              as GetHistorySrv
from kineverse_msgs.srv import GetHistoryResponse      as GetHistoryResponseMsg
from kineverse_msgs.srv import GetConstraints          as GetConstraintsSrv
from kineverse_msgs.srv import GetConstraintsResponse  as GetConstraintsResponseMsg
from kineverse_msgs.srv import ListPaths               as ListPathsSrv
from kineverse_msgs.srv import ListPathsResponse       as ListPathsResponseMsg
from kineverse_msgs.srv import SaveModel               as SaveModelSrv
from kineverse_msgs.srv import SaveModelResponse       as SaveModelResponseMsg
from kineverse_msgs.srv import LoadModel               as LoadModelSrv
from kineverse_msgs.srv import LoadModelResponse       as LoadModelResponseMsg


# Server without any ROS attachements for testing
class ModelServer_NoROS(object):
    def __init__(self, model_type=None, *args):
        self.km = model_type(*args) if model_type is not None else EventModel()

        self._changed_set = {}
        self._changed_constraints = set()
        self.lock = RLock()

    @profile
    def process_operations_msgs(self, operation_msgs):
        batch = []
        changed_ops = {}
        removed_ops = set()
        for call_msg in operation_msgs:
            if call_msg.call_mode == OperationCallMsg.REMOVE:
                batch.append(RemoveOp(call_msg.tag))
                removed_ops.add(call_msg.tag)
            else: 
                changed_ops[call_msg.tag] = call_msg.operation
                removed_ops.discard(call_msg.tag)
                instr = decode_operation_instruction(call_msg)
                for p in instr.op._root_set.values():
                    if p[0] not in self._changed_set:
                        self._changed_set[p[0]] = PathDict(False, default_factory=lambda: False)
                    self._changed_set[p[0]][p[1:]] = True
                # FIXME: self._changed_constraints.update(op._root_set.values())
                batch.append(instr)

        self.km.clean_structure()
        self.km.apply_instructions(batch)
        self.km.clean_structure()

        model_update_msg = ModelUpdateMsg()
        for p, d in self._changed_set.items():
            self._write_updates(Path(p), d, model_update_msg)

        for k in self._changed_constraints:
            if self.km.has_constraint(k):
                model_update_msg.update.constraints.append(encode_constraint(k, km.get_constraint(k)))
            else:
                cm.deleted_constraints.append(k)

        self._changed_set         = {}
        self._changed_constraints = {}

        operations_update_msg = OperationsUpdateMsg()
        for tag, op in changed_ops.items():
            stamp = self.km.get_tag_stamp(tag)
            operations_update_msg.tags.append(tag)
            operations_update_msg.stamps.append(stamp)
            operations_update_msg.operations.append(op)

        operations_update_msg.deleted = list(removed_ops)

        model_update_msg.stamp      = Time.now()
        operations_update_msg.stamp = model_update_msg.stamp
        self.km.dispatch_events()
        self._publish_updates(operations_update_msg, model_update_msg)

    def _write_updates(self, path, path_dict, model_update_msg):
        if path_dict.value:
            if self.km.has_data(path):
                model_update_msg.update.paths.append(str(path))
                model_update_msg.update.data.append(json.dumps(self.km.get_data(path)))
            else:
                model_update_msg.deleted_paths.append(str(path))
        else:
            for a, pd in path_dict.items():
                self._write_updates(path + (a,), pd, model_update_msg)

    def _publish_updates(self, operations_update_msg, model_update_msg):
        raise NotImplementedError

    @profile
    def srv_get_model(self, req):
        res = GetModelResponseMsg()
        with self.lock:
            for p in req.paths:
                if self.km.has_data(p):
                    res.model.paths.append(p)
                    res.model.data.append(json.dumps(self.km.get_data(p)))

        return res

    @profile
    def srv_get_constraints(self, req):
        res = GetConstraintsResponseMsg()
        with self.lock:
            for k, c in self.km.get_constraints_by_symbols({se.Symbol(s) for s in req.symbols}).items():
                res.constraints.append(encode_constraint(k, c))
        return res

    def srv_get_history(self, req):
        res = GetHistoryResponseMsg()
        res.stamp = Time.now()

        with self.lock:
            res.history = encode_operation_update(res.stamp, self.km.get_history_of([Path(x) for x in req.paths]))
        return res

    @profile
    def srv_apply_operations(self, req):
        res = ApplyOperationsResponseMsg()
        res.success = False
        print('Received new operation instructions.')
        try:
            with self.lock:
                self.process_operations_msgs(req.operations)
                res.success = True
                print('Done processing.')
        except Exception as e:
            print(traceback.format_exc())
            res.error_msg = str(e)

        return res


    def srv_debug_info(self, req):
        prefix = res_pkg_path(req.path)

        stamp  = datetime.date.today()
        with self.lock:
            plot_graph(generate_dependency_graph(self.km), f'{prefix}/{stamp}_dep_graph.pdf')
            plot_graph(generate_modifications_graph(self.km), f'{prefix}/{stamp}_mod_graph.pdf')

        out = DebugInfoResponseMsg()
        out.prefix = str(stamp)
        return out

    def srv_list_paths(self, req):
        res = ListPathsResponseMsg()
        
        data_root = self.km.data_tree.data_tree
        if req.root != '':
            with self.lock:
                if self.km.has_data(req.root):
                    data_root = self.km.get_data(req.root)
                else:
                    raise Exception('Data "{}" does not exist.'.format(req.root))

        depth = req.depth if req.depth > 0 else 10000
        with self.lock:
            res.paths = sorted([str(p) for p in collect_paths(data_root, Path(req.root), depth)])
        return res

    def srv_save_model(self, req):
        res = SaveModelResponseMsg(success=False)

        try:
            with open(res_pkg_path(req.filepath), 'w') as f:
                self.km.save_to_file(f, Path(req.modelpath))
                res.success = True
        except Exception as e:
            print(traceback.format_exc())
            res.error_msg = str(e)
        return res

    def srv_load_model(self, req):
        res = LoadModelResponseMsg(success=False)

        try:
            with open(res_pkg_path(req.filepath), 'r') as f:
                self.km.load_from_file(f)
            res.success = True
        except Exception as e:
            print(traceback.format_exc())
            res.error_msg = str(e)
        return res


# Server with ROS-topics and services
class ModelServer(ModelServer_NoROS):
    def __init__(self, model_type=None, *args):
        super(ModelServer, self).__init__(model_type, *args)


        self.pub_model = rospy.Publisher(stdn.topic_update_model, ModelUpdateMsg, queue_size=1)
        self.pub_ops   = rospy.Publisher(stdn.topic_update_ops, OperationsUpdateMsg, queue_size=1)

        self.services  = [
            rospy.Service(stdn.srv_apply_op, ApplyOperationsSrv, self.srv_apply_operations),
            rospy.Service(stdn.srv_get_model,        GetModelSrv,        self.srv_get_model),
            rospy.Service(stdn.srv_get_constraints,  GetConstraintsSrv,  self.srv_get_constraints),
            rospy.Service(stdn.srv_get_history,      GetHistorySrv,      self.srv_get_history),
            rospy.Service(stdn.srv_debug_info,       DebugInfoSrv,       self.srv_debug_info),
            rospy.Service(stdn.srv_list_paths,       ListPathsSrv,       self.srv_list_paths),
            rospy.Service(stdn.srv_save_model,       SaveModelSrv,       self.srv_save_model),
            rospy.Service(stdn.srv_load_model,       LoadModelSrv,       self.srv_load_model)
            ]

    def _publish_updates(self, operations_update_msg, model_update_msg):
        self.pub_ops.publish(operations_update_msg)
        self.pub_model.publish(model_update_msg)
