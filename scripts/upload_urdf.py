#!/usr/bin/env python
import rospy
import sys

from kineverse.model.event_model          import EventModel
from kineverse.network.operations_client  import OperationsClient
from kineverse.operations.urdf_operations import load_urdf
from kineverse.urdf_fix                   import urdf_filler
from kineverse.utils                      import res_pkg_path

from urdf_parser_py.urdf import URDF


if __name__ == '__main__':
    rospy.init_node('kineverse_urdf_uploader')


    if len(sys.argv) < 2:
        urdf_model = URDF.from_xml_string(rospy.get_param('/robot_description'))
    else:
        urdf_model = URDF.from_xml_file(res_pkg_path(sys.argv[1]))

    # Fill collision geometry of demanded
    if len(sys.argv) >= 3 and (sys.argv[2].lower() == 'true' or sys.argv[2] == '1'):
        urdf_model = urdf_filler(urdf_model)

    op_client = OperationsClient(EventModel, True)

    load_urdf(op_client, urdf_model.name, urdf_model)

    op_client.apply_changes()

    op_client.kill()
