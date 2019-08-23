import unittest as ut

import kineverse.network.ros_conversion as rc

from kineverse.operations.urdf_operations import load_urdf
from kineverse.network.operations_client  import OperationsClient_NoROS
from kineverse.network.ros_conversion     import encode_operation_instruction
from kineverse.network.server             import ModelServer_NoROS
from kineverse.utils                      import res_pkg_path, bb

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


class TestServer(ut.TestCase):

    def test_apply_operation(self):
        self.ops_msg   = None
        self.model_msg = None

        def cb_update(ops, model):
            self.ops_msg   = ops
            self.model_msg = model


        server = ModelServerForTesting(cb_update)
        client = OperationsClientForTesting(None, True)

        urdf_model = URDF.from_xml_file(res_pkg_path('package://kineverse/urdf/testbot.urdf'))

        load_urdf(client, urdf_model.name, urdf_model)

        op_list = client.apply_changes()

        req = bb(operations=op_list)

        server.srv_apply_operations(req)

        server.srv_apply_operations(req)

        client.cb_operations_update(self.ops_msg)        


if __name__ == '__main__':
    ut.main()