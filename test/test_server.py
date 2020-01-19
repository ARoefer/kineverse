import unittest as ut

import kineverse.network.ros_conversion as rc
import kineverse.json_wrapper           as json

from kineverse.operations.urdf_operations import load_urdf
from kineverse.model.paths                import Path
from kineverse.network.operations_client  import OperationsClient_NoROS
from kineverse.network.model_client       import ModelClient_NoROS
from kineverse.network.ros_conversion     import encode_operation_instruction
from kineverse.network.server             import ModelServer_NoROS
from kineverse.utils                      import res_pkg_path, bb

from kineverse.msg import Model       as ModelMsg
from kineverse.msg import ModelUpdate as ModelUpdateMsg

from kineverse.srv import ApplyOperationsRequest as ApplyOperationsRequestMsg

from urdf_parser_py.urdf import URDF

# Just implements the update function to return the generated messages
class ModelServerForTesting(ModelServer_NoROS):
    def __init__(self, cb_update):
        super(ModelServerForTesting, self).__init__()
        self.__cb_update = cb_update

    def _publish_updates(self, operations_update_msg, model_update_msg):
        self.__cb_update(operations_update_msg, model_update_msg)


class OperationsClientForTesting(OperationsClient_NoROS):
    def apply_changes(self):
        out = [encode_operation_instruction(x) for x in self._operations_batch]
        self._operations_batch = []        
        return out


class ModelClientForTesting(ModelClient_NoROS):
    def __init__(self, srv_get_model, srv_get_constraints, model_type, *args):
        super(ModelClientForTesting, self).__init__(model_type, *args)
        self.srv_get_model       = srv_get_model
        self.srv_get_constraints = srv_get_constraints


class TestServer(ut.TestCase):

    def test_apply_operation(self):
        self.ops_msg   = None
        self.model_msg = None

        def cb_update(ops, model):
            self.ops_msg   = ops
            self.model_msg = model

        server = ModelServerForTesting(cb_update)

        def cb_srv_get_model(paths, csymbs):
            return server.srv_get_model(bb(paths=paths, constrained_symbols=csymbs))

        def cb_srv_get_constraints(csymbs):
            return server.srv_get_constraints(bb(symbols=csymbs))

        op_client = OperationsClientForTesting(None, True)
        m_client  = ModelClientForTesting(cb_srv_get_model, cb_srv_get_constraints, None)
        m_client.has_data('testbot')

        urdf_model = URDF.from_xml_file(res_pkg_path('package://kineverse/urdf/testbot.urdf'))

        load_urdf(op_client, urdf_model.name, urdf_model)

        op_list = op_client.apply_changes()

        req = bb(operations=op_list)

        server.srv_apply_operations(req)
        server.srv_apply_operations(req)

        op_client.cb_operations_update(self.ops_msg)
        
        with open('update.json', 'w') as f:
            json.dump(dict(zip(self.model_msg.update.paths, 
                               self.model_msg.update.data)), f, indent=2)

        m_client.cb_model_update(self.model_msg)
        o_model = server.km.get_data('testbot')
        t_model = m_client.get_data('testbot')

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

if __name__ == '__main__':
    ut.main()