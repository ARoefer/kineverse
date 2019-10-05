#!/usr/bin/env python
import rospy
import sys

from kineverse.gradients.gradient_math    import vector3
from kineverse.model.event_model          import EventModel, Path
from kineverse.model.frames                  import Frame
from kineverse.network.operations_client  import OperationsClient
from kineverse.operations.basic_operations import CreateComplexObject
from kineverse.operations.urdf_operations  import load_urdf
from kineverse.operations.special_kinematics import create_roomba_joint_with_symbols
from kineverse.urdf_fix                   import urdf_filler
from kineverse.utils                      import res_pkg_path

from urdf_parser_py.urdf import URDF

if __name__ == '__main__':
    rospy.init_node('kineverse_roomba_uploader')


    if len(sys.argv) < 2:
        urdf_model = URDF.from_xml_string(rospy.get_param('/robot_description'))
    else:
        urdf_model = URDF.from_xml_file(res_pkg_path(sys.argv[1]))

    # Fill collision geometry of demanded
    if len(sys.argv) >= 3 and (sys.argv[2].lower() == 'true' or sys.argv[2] == '1'):
        urdf_model = urdf_filler(urdf_model)

    op_client = OperationsClient(EventModel, True)

    load_urdf(op_client, urdf_model.name, urdf_model)
    op_client.apply_operation_before('create map', 'create {}'.format(urdf_model.name), CreateComplexObject(Path('map'), Frame('')))

    roomba_op = create_roomba_joint_with_symbols(Path('map/pose'), 
                                                 Path('{}/links/base_link/pose'.format(urdf_model.name)),
                                                 Path('{}/joints/to_map'.format(urdf_model.name)),
                                                 vector3(0,0,1),
                                                 vector3(1,0,0),
                                                 1.0, 0.6, Path(urdf_model.name))
    op_client.apply_operation_after('connect map base_link', 'create {}/base_link'.format(urdf_model.name), roomba_op)

    op_client.apply_changes()

    op_client.kill()