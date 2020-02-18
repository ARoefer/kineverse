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

    def __eq__(self, other):
        if isinstance(other, RoombaJoint):
            return super(RoombaJoint, self).__eq__(other) and self.x_pos == other.x_pos and self.y_pos == other.y_pos and self.a_pos == other.a_pos and self.a_pos == other.a_pos and self.lin_vel == other.lin_vel and self.ang_vel == other.ang_vel
        return False


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


class OmnibaseJoint(KinematicJoint):
    def __init__(self, parent, child, x_pos, y_pos, a_pos):
        super(OmnibaseJoint, self).__init__('omni', parent, child)
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.a_pos = a_pos

    def _json_data(self, json_dict):
        super(OmnibaseJoint, self)._json_data(json_dict)
        del json_dict['jtype']
        json_dict.update({'x_pos'   : self.x_pos,
                          'y_pos'   : self.y_pos,
                          'a_pos'   : self.a_pos})

    def __eq__(self, other):
        if isinstance(other, OmnibaseJoint):
            return super(OmnibaseJoint, self).__eq__(other) and self.x_pos == other.x_pos and self.y_pos == other.y_pos and self.a_pos == other.a_pos
        return False


class SetOmnibaseJoint(Operation):
    def init(self, parent_pose, child_pose, connection_path, rot_axis, x_pos, y_pos, a_pos, l_vel_limit, a_vel_limit):
        self.joint_obj = OmnibaseJoint(str(parent_pose[:-1]), str(child_pose[:-1]), x_pos, y_pos, a_pos)
        self.conn_path = connection_path
        op_construction_wrapper(super(SetOmnibaseJoint, self).init,
                                'Roomba Joint', 
                                ['child_pose', 'child_parent', 'child_parent_tf'],
                                (connection_path, 'connection', self.joint_obj),
                                parent_pose=parent_pose,
                                child_parent=child_pose[:-1] + ('parent',),
                                child_parent_tf=child_pose[:-1] + ('to_parent',),
                                child_pose=child_pose,
                                rot_axis=rot_axis,
                                x_pos=x_pos,
                                y_pos=y_pos,
                                a_pos=a_pos,
                                lin_vel_limit=l_vel_limit,
                                ang_vel_limit=a_vel_limit)

    def _apply(self, ks, parent_pose, child_pose, child_parent_tf, rot_axis, x_pos, y_pos, a_pos, lin_vel_limit, ang_vel_limit):
        child_parent_tf = translation3(x_pos, y_pos, 0) * rotation3_axis_angle(rot_axis, a_pos)
        return {'child_parent': self.joint_obj.parent,
                'child_parent_tf': child_parent_tf,
                'child_pose': parent_pose * child_parent_tf,
                'connection': self.joint_obj}, \
               {'{}_lin_vel'.format(self.conn_path): Constraint(-lin_vel_limit, lin_vel_limit, get_diff(norm(vector3(x_pos, y_pos, 0)))),
                '{}_ang_vel'.format(self.conn_path): Constraint(-ang_vel_limit, ang_vel_limit, get_diff(a_pos))}

def create_omnibase_joint_with_symbols(parent_pose, child_pose, connection_path, rot_axis, lin_vel_limit, ang_vel_limit, var_prefix):
    return SetOmnibaseJoint(parent_pose, child_pose, connection_path, rot_axis,
                create_pos((var_prefix + ('localization_x',)).to_symbol()),
                create_pos((var_prefix + ('localization_y',)).to_symbol()),
                create_pos((var_prefix + ('localization_a',)).to_symbol()),
                lin_vel_limit, ang_vel_limit)


class BallJoint(KinematicJoint):
    def __init__(self, parent, child, axis_x, axis_y, axis_z):
        super(BallJoint, self).__init__('ball', parent, child)
        self.axis_x = axis_x
        self.axis_y = axis_y
        self.axis_z = axis_z

    def __eq__(self, other):
        if isinstance(other, BallJoint):
            return super(BallJoint, self).__eq__(other) and self.axis_x == other.axis_x and self.axis_y == other.axis_y and self.axis_z == other.axis_z
        return False


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

        pos_measure = dot(vector3(0,0,1), normed_axis)
        limit_dot_space = cos(pos_limit)

        return {'child_parent': self.joint_obj.parent,
                'child_parent_tf': connection_tf * child_parent_tf,
                'child_pose': parent_pose * connection_tf * rot_matrix * child_pose,
                'connection': self.joint_obj}, \
               {'{}_position'.format(self.conn_path): Constraint(limit_dot_space - pos_measure, 1, get_diff(pos_measure)),
                '{}_velocity'.format(self.conn_path): Constraint(-vel_limit, vel_limit, get_diff(norm(rot_axis)))}


class TwoDOFRotationJoint(KinematicJoint):
    def __init__(self, parent, child, axis_x, axis_y):
        super(TwoDOFRotationJoint).__init__('2dof', parent, child)
        self.axis_x = axis_x
        self.axis_y = axis_y

    def __eq__(self, other):
        if isinstance(other, TwoDOFRotationJoint):
            return super(TwoDOFRotationJoint, self).__eq__(other) and self.axis_x == other.axis_x and self.axis_y == other.axis_y
        return False

# class Set2DofRotationJoint(Operation):
#     def init(self, parent_pose, child_pose, connection_path, connection_tf, axis_x, axis_y, pos)
