#!/usr/bin/env python
import rospy
import sys

from kineverse.gradients.gradient_math       import vector3
from kineverse.model.paths                   import Path, find_all
from kineverse.model.event_model             import EventModel
from kineverse.model.frames                  import Frame, get_root_frames
from kineverse.network.operations_client     import OperationsClient
from kineverse.operations.basic_operations   import CreateComplexObject
from kineverse.operations.urdf_operations    import load_urdf
from kineverse.operations.special_kinematics import create_omnibase_joint_with_symbols
from kineverse.urdf_fix                      import urdf_filler
from kineverse.utils                         import res_pkg_path

from urdf_parser_py.urdf import URDF

if __name__ == '__main__':
    rospy.init_node('kineverse_omnibase_uploader')


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

    omni_op = create_omnibase_joint_with_symbols(Path('map/pose'), 
                                                 Path('{}/links/{}/pose'.format(urdf_model.name, urdf_model.get_root())),
                                                 Path('{}/joints/to_map'.format(urdf_model.name)),
                                                 vector3(0,0,1),
                                                 1.0, 0.6, Path(urdf_model.name))
    op_client.apply_operation_after('connect map {}'.format(urdf_model.get_root()), 'create {}/{}'.format(urdf_model.name, urdf_model.get_root()), omni_op)

    op_client.apply_changes()

    op_client.kill()
