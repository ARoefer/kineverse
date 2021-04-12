import unittest as ut
import math

import kineverse.network.ros_conversion as rc
import kineverse.json_wrapper           as json

from kineverse.time_wrapper                import Time
from kineverse.operations.urdf_operations  import load_urdf
from kineverse.operations.basic_operations import CreateValue, ExecFunction
from kineverse.model.paths                 import Path
from kineverse.network.operations_client   import OperationsClient
from kineverse.network.model_client        import ModelClient_NoROS
from kineverse.network.ros_conversion      import encode_operation_instruction, encode_operation_update
from kineverse.network.server              import ModelServer_NoROS
from kineverse.utils                       import res_pkg_path, bb

from kineverse_msgs.msg import Model       as ModelMsg
from kineverse_msgs.msg import ModelUpdate as ModelUpdateMsg

from kineverse_msgs.srv import ApplyOperationsRequest as ApplyOperationsRequestMsg, \
                               GetHistoryResponse     as GetHistoryResponseMsg

from urdf_parser_py.urdf import URDF

# Just implements the update function to return the generated messages
class ModelServerForTesting(ModelServer_NoROS):
    def __init__(self, cb_update):
        super(ModelServerForTesting, self).__init__()
        self.__cb_update = cb_update

    def _publish_updates(self, operations_update_msg, model_update_msg):
        self.__cb_update(operations_update_msg, model_update_msg)


class OperationsClientForTesting(OperationsClient):
    def __init__(self, server, model_type, *args):
        super(OperationsClientForTesting, self).__init__(model_type, *args)
        self._test_server = server

        def srv_apply_operation(instr_msg):
            self._test_server.srv_apply_operations(bb(operations=[instr_msg]))

        self.srv_apply_operation = srv_apply_operation

    def apply_changes(self):
        req = ApplyOperationsRequestMsg()
        req.operations = [encode_operation_instruction(x) for x in self._operations_batch]
        self._operations_batch = []        
        self._test_server.srv_apply_operations(req)

    def srv_get_history(self, req):
        res = GetHistoryResponseMsg()
        stamp = Time.now()

        res.history = encode_operation_update(stamp, self.km.get_history_of([Path(x) for x in req]))
        return res



class ModelClientForTesting(ModelClient_NoROS):
    def __init__(self, srv_get_model, srv_get_constraints, model_type, *args):
        super(ModelClientForTesting, self).__init__(model_type, *args)
        self.srv_get_model       = srv_get_model
        self.srv_get_constraints = srv_get_constraints


def sin(x):
    return math.sin(x)

def my_abs(x):
    return abs(x)


class TestServer(ut.TestCase):

    def test_apply_operation(self):
        self.ops_msg   = None
        self.model_msg = None
        self.m_client  = None
        self.op_client = None

        def cb_update(ops, model):
            self.ops_msg   = ops
            self.model_msg = model
            self.op_client.cb_operations_update(self.ops_msg)
            self.m_client.cb_model_update(self.model_msg)

        server = ModelServerForTesting(cb_update)

        def cb_srv_get_model(paths, csymbs):
            return server.srv_get_model(bb(paths=paths, constrained_symbols=csymbs))

        def cb_srv_get_constraints(csymbs):
            return server.srv_get_constraints(bb(symbols=csymbs))

        self.op_client = OperationsClientForTesting(server, None, True)
        self.m_client  = ModelClientForTesting(cb_srv_get_model, cb_srv_get_constraints, None)
        self.m_client.has_data('testbot')

        urdf_model = URDF.from_xml_file(res_pkg_path('package://kineverse/urdf/testbot.urdf'))

        load_urdf(self.op_client, urdf_model.name, urdf_model)

        op_list = self.op_client.apply_changes()

        req = bb(operations=op_list)

        server.srv_apply_operations(req)
        server.srv_apply_operations(req)

        with open('update.json', 'w') as f:
            json.dump(dict(zip(self.model_msg.update.paths,
                               self.model_msg.update.data)), f, indent=2)

        o_model = server.km.get_data('testbot')
        t_model = self.m_client.get_data('testbot')

        with open('original.json', 'w') as f:
            json.dump(o_model, f, indent=True)

        with open('transmitted.json', 'w') as f:
            json.dump(t_model, f, indent=True)

        self.assertEquals(o_model.name, t_model.name)
        
        l  = o_model.links['arm_mount_link']
        lt = t_model.links['arm_mount_link']

        for n, l in o_model.links.items():
            if n not in t_model.links:
                print('Link {} missing from transmitted model'.format(n))
                continue
            lt = t_model.links[n]
        if l == lt:
            print('Equality check passed for link {}'.format(n))
            print('Types: {} {}'.format(type(l), type(lt)))
            print('parent: {}'.format(lt.parent == l.parent))
            print('pose: {}'.format(lt.pose == l.pose))
            print('to_parent: {}'.format(lt.to_parent == l.to_parent))
            print('geometry: {}'.format(lt.geometry == l.geometry))
            print('collision: {}'.format(lt.collision == l.collision))
            print('inertial: {}\n---'.format(lt.inertial == l.inertial))
        else:
            print('Equality check failed for link {}'.format('arm_mount_link'))
            print('Types: {} {}'.format(type(l), type(lt)))
            print('parent: {}'.format(lt.parent == l.parent))
            print('pose: {}\n{}'.format(lt.pose == l.pose, l.pose - lt.pose))
            print('to_parent: {}\n{}'.format(lt.to_parent == l.to_parent, l.to_parent - lt.to_parent))
            print('geometry: {}'.format(lt.geometry == l.geometry))
            print('collision: {}'.format(lt.collision == l.collision))
            print('inertial: {}'.format(lt.inertial == l.inertial))

        self.assertEquals(o_model.joints, t_model.joints)
        self.assertEquals(o_model, t_model)

    def test_new_operations_client(self):
        
        def cb_update(ops_msg, model_msg):
            pass

        server = ModelServerForTesting(cb_update)

        cop_1 = OperationsClientForTesting(server, None)
        cop_2 = OperationsClientForTesting(server, None)

        cop_1.apply_operation('create a', CreateValue(Path('a'), 4.0))
        cop_1.apply_operation('create sin(a)', ExecFunction(Path('sin_a'), sin, Path('a')))
        cop_1.apply_operation('create abs(sin(a))', ExecFunction(Path('abs_sin_a'), 
                                                                 my_abs, 
                                                                 Path('sin_a')))

        self.assertTrue(server.km.has_data('a'))
        self.assertTrue(server.km.has_data('sin_a'))
        self.assertTrue(server.km.has_data('abs_sin_a'))
        self.assertTrue(server.km.has_tag('create a'))
        self.assertTrue(server.km.has_tag('create sin(a)'))
        self.assertTrue(server.km.has_tag('create abs(sin(a))'))


if __name__ == '__main__':
    ut.main()
