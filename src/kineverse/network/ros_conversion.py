import traceback
import betterpybullet as pb

import kineverse.json_wrapper as json
import kineverse.type_sets as ts

from kineverse.gradients.gradient_math import spw
from kineverse.model.kinematic_model import ApplyAt, ApplyBefore, ApplyAfter, RemoveOp, Constraint
from kineverse.utils                 import import_class, real_quat_from_matrix

from kineverse.msg import OperationCall    as OperationCallMsg
from kineverse.msg import Operation        as OperationMsg
from kineverse.msg import Constraint       as ConstraintMsg
from kineverse.msg import OperationsUpdate as OperationsUpdateMsg

from std_msgs.msg      import Header, String, Float64, Bool, Int32
from geometry_msgs.msg import Pose    as PoseMsg
from geometry_msgs.msg import Point   as PointMsg
from geometry_msgs.msg import Vector3 as Vector3Msg
from geometry_msgs.msg import Quaternion as QuaternionMsg
from geometry_msgs.msg import PoseStamped as PoseStampedMsg

operations_registry = {}
call_mapping = {ApplyAt:     OperationCallMsg.INSERT_AT, 
                ApplyBefore: OperationCallMsg.INSERT_BEFORE,
                ApplyAfter:  OperationCallMsg.INSERT_AFTER,
                RemoveOp:    OperationCallMsg.REMOVE}
inv_call_mapping = {v: t for t, v in call_mapping.items()}

def encode_operation_update(stamp, tagged_ops):
    out = OperationsUpdateMsg()
    out.stamp = stamp
    out.tags, out.stamps, out.operations = zip(*[(to.tag, to.stamp, encode_operation(to.op)) for to in tagged_ops])
    return out

def decode_op_msg(msg):
    if msg.operation_type not in operations_registry:
        if msg.operation_type[:8] == '<class \'' and msg.operation_type[-2:] == '\'>':
            operations_registry[msg.operation_type] = import_class(msg.operation_type[8:-2])
        else:
            raise Exception('Operation type "{}" does not match the pattern "<class \'TYPE\'>"'.format(msg.operation_type))
    params = []
    for x in msg.parameters:
        try:
            params.append(json.loads(x))
        except Exception as e:
            raise Exception('An exception was raised during the parsing of parameters of the operations message of type {}. Parameter JSON:\n{}\nError: {}\n{}'.format(msg.operation_type, x, str(e), traceback.format_exc()))
    return operations_registry[msg.operation_type](*[json.loads(x) for x in msg.parameters])

def encode_operation(operation):
    out = OperationMsg()
    out.operation_type = str(type(operation))
    out.parameters = [json.dumps(x) for x in operation._construction_args]
    return out

def encode_operation_instruction(op_instr):
    out = OperationCallMsg()
    out.tag       = op_instr.tag
    out.call_mode = call_mapping[type(op_instr)]
    out.reference_tag = op_instr.ref_tag if type(op_instr) == ApplyBefore or type(op_instr) == ApplyAfter else ''
    out.operation = encode_operation(op_instr.op)
    return out

def decode_operation_instruction(instr_msg):
    if instr_msg.call_mode not in inv_call_mapping:
        raise Exception('Unrecognized call mode in operations message: {}'.format(instr_msg.call_mode))

    t = inv_call_mapping[instr_msg.call_mode]
    if t is ApplyAfter or t is ApplyBefore:
        return t(decode_op_msg(instr_msg.operation), instr_msg.tag, instr_msg.reference_tag)
    elif t is RemoveOp:
        return t(instr_msg.tag)
    return t(decode_op_msg(instr_msg.operation), instr_msg.tag)


def decode_constraint(msg):
    return Constraint(json.loads(msg.lower), 
                      json.loads(msg.upper), 
                      json.loads(msg.expr))

def decode_constraint_filtered(msg, symbol_set):
    expr = json.loads(msg.expr)
    if len(symbol_set.intersection(expr.free_symbols)) > 0:
        return Constraint(json.loads(msg.lower), 
                          json.loads(msg.upper), 
                          expr)
    return None


def encode_constraint(name, constraint):
    out = ConstraintMsg()
    out.name  = name
    out.lower = json.dumps(constraint.lower)
    out.upper = json.dumps(constraint.upper)
    out.expr  = json.dumps(constraint.expr)    
    return out

def encode_point(iterable):
    out = PointMsg()
    out.x = iterable[0]
    out.y = iterable[1]
    out.z = iterable[2]
    return out

def encode_vector(iterable):
    out = Vector3Msg()
    out.x = iterable[0]
    out.y = iterable[1]
    out.z = iterable[2]
    return out

def encode_rotation(data):
    quat = real_quat_from_matrix(data)
    out = QuaternionMsg()
    out.w = quat[3]
    out.x = quat[0]
    out.y = quat[1]
    out.z = quat[2]
    return out

def encode_pose(data):
    out = PoseMsg()
    out.position = encode_point(data[:, 3:])
    out.orientation = encode_rotation(data[:3, :3])
    return out

def auto_encode(data):
    t = type(data)
    if t in ts.matrix_types:
        m = data if t in ts.symengine_matrix_types else data.to_sym_matrix()
        if m.shape == (4, 4):
            return encode_pose(m)
        elif m.shape == (3, 3):
            return encode_rotation(m)
        elif m.shape == (3, 1):
            return encode_point(m)
        elif m.shape == (4, 1):
            return encode_vector(m) if m[3] == 0 else encode_point(m)
        else:
            raise Exception('No encoding known for matrices of shape {}'.format(m.shape))
    elif t == list or t == tuple:
        if len(data) == 0:
            raise Exception('Can not encode empty list.')

        s_t = type(data[0])
        if s_t is float or s_t is int or s_t is list or s_t is tuple:
            return auto_encode(spw.Matrix(data))
        raise Exception('Can not encode list with inner type {}'.format(s_t))
    elif t == pb.Transform:
        out = PoseMsg()
        out.position.x = data.origin.x
        out.position.y = data.origin.y
        out.position.z = data.origin.z
        out.orientation.x = data.rotation.x
        out.orientation.y = data.rotation.y
        out.orientation.z = data.rotation.z
        out.orientation.w = data.rotation.w
        return out
    elif t == pb.Quaternion:
        out = QuaternionMsg()
        out.x = data.x
        out.y = data.y
        out.z = data.z
        out.w = data.w
        return out
    elif t == pb.Vector3:
        out = PointMsg()
        out.x = data.x
        out.y = data.y
        out.z = data.z
        return out
    else:
        raise Exception('Can not encode data of type {}'.format(t))
