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

    def _json_data(self, json_dict):
        super(RoombaJoint, self)._json_data(json_dict)
        del json_dict['jtype']
        json_dict.update({'x_pos'   : self.x_pos,
                          'y_pos'   : self.y_pos,
                          'z_pos'   : self.z_pos,
                          'a_pos'   : self.a_pos,
                          'lin_vel' : self.lin_vel,
                          'ang_vel' : self.ang_vel})


class SetRoombaJoint(Operation):
    def init(self, parent_pose, child_pose, connection_path, rot_axis, lin_axis, x_pos, y_pos, z_pos, a_pos, lin_vel, ang_vel, l_vel_limit, a_vel_limit):
        self.joint_obj = RoombaJoint(str(parent_pose[:-1]), str(child_pose[:-1]), x_pos, y_pos, z_pos, a_pos, lin_vel, ang_vel)
        self.conn_path = connection_path
        op_construction_wrapper(super(SetRoombaJoint, self).init,
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
                create_pos((var_prefix + ('localization_x',)).to_symbol()),
                create_pos((var_prefix + ('localization_y',)).to_symbol()),
                create_pos((var_prefix + ('localization_z',)).to_symbol()),
                create_pos((var_prefix + ('localization_a',)).to_symbol()),
                create_vel((var_prefix + ('linear_joint',)).to_symbol()),
                create_vel((var_prefix + ('angular_joint',)).to_symbol()),
                lin_vel_limit, ang_vel_limit)
                lin_vel_limit, ang_vel_limit)


class BallJoint(KinematicJoint):
    def __init__(self, parent, child, axis_x, axis_y, axis_z):
        super(BallJoint, self).__init__('ball', parent, child)
        self.axis_x = axis_x
        self.axis_y = axis_y
        self.axis_z = axis_z


class SetBallJoint(Operation):
    def init(self, parent_pose, child_pose, connection_path, connection_tf, axis_x, axis_y, axis_z, pos_limit, vel_limit):
        self.joint_obj = BallJoint(str(parent_pose[:-1]), str(child_pose[:-1]), axis_x, axis_y, axis_z)
        self.conn_path = connection_path
        op_construction_wrapper(super(SetBallJoint, self).init,
                                'Ball Joint',
                                ['child_pose', 'child_parent', 'child_parent_tf'],
                                (connection_path, 'connection', self.joint_obj),
                                parent_pose=parent_pose,
                                child_pose=child_pose,
                                child_parent=child_pose[:-1] + ('parent',),
                                child_parent_tf=child_pose[:-1] + ('to_parent',),
                                connection_tf=connection_tf,
                                axis_x=axis_x,
                                axis_y=axis_y,
                                axis_z=axis_z,
                                pos_limit=pos_limit,
                                vel_limit=vel_limit)

        def _apply(self, km, parent_pose, child_pose, child_parent, child_parent_tf, connection_tf, axis_x, axis_y, axis_z, pos_limit, vel_limit):
            rot_axis = vector3(axis_x, axis_y, axis_z)
            normed_axis = rot_axis / (norm(rot_axis) + 1e-4)
            rot_matrix = rotation3_axis_angle(normed_axis, norm(rot_axis))

            pos_measure = dot(unitZ, normed_axis)
            limit_dot_space = cos(pos_limit)

            return {'child_parent': self.joint_obj.parent,
                    'child_parent_tf': connection_tf * child_parent_tf,
                    'child_pose': parent_pose * connection_tf * rot_matrix * child_pose,
                    'connection': self.joint_obj}, \
                   {'{}_position'.format(self.conn_path): Constraint(limit_dot_space - pos, 1, get_diff(pos_measure)),
                    '{}_velocity'.format(self.conn_path): Constraint(-vel_limit, vel_limit, get_diff(norm(rot_axis)))}


class TwoDOFRotationJoint(KinematicJoint):
    def __init__(self, parent, child, axis_x, axis_y):
        super(TwoDOFRotationJoint).__init__('2dof', parent, child)
        self.axis_x = axis_x
        self.axis_y = axis_y


# class Set2DofRotationJoint(Operation):
#     def init(self, parent_pose, child_pose, connection_path, connection_tf, axis_x, axis_y, pos)