from kineverse.gradients.diff_logic       import create_symbol, TYPE_POSITION, TYPE_VELOCITY
from kineverse.gradients.gradient_math    import *
from kineverse.model.kinematic_model      import Constraint
from kineverse.operations.operation       import Operation, op_construction_wrapper
from kineverse.operations.urdf_operations import KinematicJoint



class RoombaJoint(KinematicJoint):
    def __init__(self, parent, child, x_pos, y_pos, z_pos, a_pos, lin_vel, ang_vel):
        super(RoombaJoint, self).__init__('roomba', parent, child)
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.z_pos = z_pos
        self.a_pos = a_pos
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel

class SetRoombaJoint(Operation):
    def __init__(self, parent_pose, child_pose, connection_path, rot_axis, lin_axis, x_pos, y_pos, z_pos, a_pos, lin_vel, ang_vel, l_vel_limit, a_vel_limit):
        self.joint_obj = RoombaJoint(str(parent_pose[:-1]), str(child_pose[:-1]), x_pos, y_pos, z_pos, a_pos, lin_vel, ang_vel)
        self.conn_path = connection_path
        op_construction_wrapper(super(SetRoombaJoint, self).__init__,
                                'Roomba Joint', 
                                ['child_pose', 'child_parent', 'child_parent_tf'],
                                (connection_path, 'connection', self.joint_obj),
                                parent_pose=parent_pose,
                                child_parent=child_pose[:-1] + ('parent',),
                                child_parent_tf=child_pose[:-1] + ('to_parent',),
                                child_pose=child_pose,
                                rot_axis=rot_axis,
                                lin_axis=lin_axis,
                                x_pos=x_pos,
                                y_pos=y_pos,
                                z_pos=z_pos,
                                a_pos=a_pos,
                                lin_vel=lin_vel,
                                ang_vel=ang_vel,
                                lin_vel_limit=l_vel_limit,
                                ang_vel_limit=a_vel_limit)

    def _apply(self, ks, parent_pose, child_pose, child_parent_tf, rot_axis, lin_axis, x_pos, y_pos, z_pos, a_pos, lin_vel, ang_vel, lin_vel_limit, ang_vel_limit):
        lin_axis  = wrap_matrix_mul(lin_axis, GC(0, {lin_vel: 1}))
        rot_pos   = GC(a_pos, {ang_vel: 1})
        child_parent_tf = frame3_axis_angle(rot_axis, rot_pos, point3(x_pos, y_pos, z_pos)) * translation3(*lin_axis[:3])
        return {'child_parent': self.joint_obj.parent,
                'child_parent_tf': child_parent_tf,
                'child_pose': parent_pose * child_parent_tf,
                'connection': self.joint_obj}, \
               {'{}_lin_vel'.format(self.conn_path): Constraint(-lin_vel_limit, lin_vel_limit, lin_vel),
                '{}_ang_vel'.format(self.conn_path): Constraint(-ang_vel_limit, ang_vel_limit, ang_vel),}        


def create_roomba_joint_with_symbols(parent_pose, child_pose, connection_path, rot_axis, lin_axis, lin_vel_limit, ang_vel_limit, var_prefix):
    return SetRoombaJoint(parent_pose, child_pose, connection_path, rot_axis, lin_axis,
                create_symbol((var_prefix + ('localization_x',)).to_symbol(), TYPE_POSITION),
                create_symbol((var_prefix + ('localization_y',)).to_symbol(), TYPE_POSITION),
                create_symbol((var_prefix + ('localization_z',)).to_symbol(), TYPE_POSITION),
                create_symbol((var_prefix + ('localization_a',)).to_symbol(), TYPE_POSITION),
                create_symbol((var_prefix + ('linear',)).to_symbol(), TYPE_VELOCITY),
                create_symbol((var_prefix + ('angular',)).to_symbol(), TYPE_VELOCITY),
                lin_vel_limit, ang_vel_limit)