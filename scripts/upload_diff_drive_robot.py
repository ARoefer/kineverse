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
from kineverse.operations.special_kinematics import create_diff_drive_joint_with_symbols
from kineverse.urdf_fix                      import urdf_filler
from kineverse.utils                         import res_pkg_path

from urdf_parser_py.urdf import URDF

from math import pi

if __name__ == '__main__':
    rospy.init_node('kineverse_roomba_uploader')

    if len(sys.argv) < 3:
        print('Usage: [urdf file] wheel_radius wheel_distance [fill_collision_tags]')
        exit()
    elif len(sys.argv) < 4:
        urdf_model = URDF.from_xml_string(rospy.get_param('/robot_description'))
        radius   = float(sys.argv[1])
        distance = float(sys.argv[2])
    else:
        urdf_model = URDF.from_xml_file(res_pkg_path(sys.argv[1]))
        radius   = float(sys.argv[2])
        distance = float(sys.argv[3])


    # Fill collision geometry if demanded
    if len(sys.argv) >= 5 and (sys.argv[4].lower() == 'true' or sys.argv[4] == '1'):
        urdf_model = urdf_filler(urdf_model)

    op_client = OperationsClient(EventModel, True)

    load_urdf(op_client, urdf_model.name, urdf_model)
    op_client.apply_operation_before('create world', 'create {}'.format(urdf_model.name), CreateComplexObject(Path('world'), Frame('')))

    r_limit = 1.0 / (pi * radius * 2)

    drive_op = create_diff_drive_joint_with_symbols(Path('world/pose'), 
                                                 Path('{}/links/{}/pose'.format(urdf_model.name, urdf_model.get_root())),
                                                 Path('{}/joints/to_world'.format(urdf_model.name)),
                                                 radius, distance, r_limit, Path(urdf_model.name))
    op_client.apply_operation_after('connect world {}'.format(urdf_model.get_root()), 'create {}/{}'.format(urdf_model.name, urdf_model.get_root()), drive_op)

    op_client.apply_changes()
    op_client.kill()
