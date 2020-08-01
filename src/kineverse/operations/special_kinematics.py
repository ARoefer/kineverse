from kineverse.gradients.diff_logic       import create_symbol, TYPE_POSITION, TYPE_VELOCITY
from kineverse.gradients.gradient_math    import *
from kineverse.model.articulation_model   import Constraint
from kineverse.operations.operation       import Operation, op_construction_wrapper
from kineverse.operations.urdf_operations import KinematicJoint



class DiffDriveJoint(KinematicJoint):
    def __init__(self, parent, child, x_pos, y_pos, z_pos, a_pos, r_wheel_vel, l_wheel_vel, wheel_radius, wheel_distance):
        super(DiffDriveJoint, self).__init__('roomba', parent, child)
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.z_pos = z_pos
        self.a_pos = a_pos
        self.r_wheel_vel = r_wheel_vel
        self.l_wheel_vel = l_wheel_vel
        self.wheel_radius = wheel_radius 
        self.wheel_distance = wheel_distance 

    def _json_data(self, json_dict):
        super(DiffDriveJoint, self)._json_data(json_dict)
        del json_dict['jtype']
        json_dict.update({'x_pos'   : self.x_pos,
                          'y_pos'   : self.y_pos,
                          'z_pos'   : self.z_pos,
                          'a_pos'   : self.a_pos,
                          'r_wheel_vel' : self.r_wheel_vel,
                          'l_wheel_vel' : self.l_wheel_vel,
                          'wheel_radius' : self.wheel_radius,
                          'wheel_distance' : self.wheel_distance})

    def __eq__(self, other):
        if isinstance(other, DiffDriveJoint):
            return super(DiffDriveJoint, self).__eq__(other) and self.x_pos == other.x_pos and self.y_pos == other.y_pos and self.a_pos == other.a_pos and self.a_pos == other.a_pos and self.r_wheel_vel == other.r_wheel_vel and self.l_wheel_vel == other.l_wheel_vel and self.wheel_radius == other.wheel_radius and self.wheel_distance == other.wheel_distance
        return False


class SetDiffDriveJoint(Operation):
    def init(self, parent_pose, child_pose, connection_path, x_pos, y_pos, z_pos, a_pos, r_wheel_vel, l_wheel_vel, wheel_vel_limit, wheel_radius, wheel_distance):
        self.joint_obj = DiffDriveJoint(str(parent_pose[:-1]), str(child_pose[:-1]), x_pos, y_pos, z_pos, a_pos, r_wheel_vel, l_wheel_vel, wheel_radius, wheel_distance)
        self.conn_path = connection_path
        op_construction_wrapper(super(SetDiffDriveJoint, self).init,
                                'DiffDrive Joint', 
                                ['child_pose', 'child_parent', 'child_parent_tf'],
                                (connection_path, 'connection', self.joint_obj),
                                parent_pose=parent_pose,
                                child_parent=child_pose[:-1] + ('parent',),
                                child_parent_tf=child_pose[:-1] + ('to_parent',),
                                child_pose=child_pose,
                                x_pos=x_pos,
                                y_pos=y_pos,
                                z_pos=z_pos,
                                a_pos=a_pos,
                                r_wheel_vel=r_wheel_vel,
                                l_wheel_vel=l_wheel_vel,
                                wheel_vel_limit=wheel_vel_limit,
                                wheel_radius=wheel_radius,
                                wheel_distance=wheel_distance)

    def _apply(self, ks, parent_pose, child_pose, child_parent_tf, x_pos, y_pos, z_pos, a_pos, r_wheel_vel, l_wheel_vel, wheel_vel_limit, wheel_radius, wheel_distance):

        pos_pos = point3(GC(x_pos, {r_wheel_vel: cos(a_pos) * wheel_radius * 0.5,
                                    l_wheel_vel: cos(a_pos) * wheel_radius * 0.5}), 
                         GC(y_pos, {r_wheel_vel: sin(a_pos) * wheel_radius * 0.5,
                                    l_wheel_vel: sin(a_pos) * wheel_radius * 0.5}), z_pos)

        rot_pos = GC(a_pos, {r_wheel_vel:   wheel_radius / wheel_distance,
                             l_wheel_vel: - wheel_radius / wheel_distance})
        child_parent_tf = frame3_axis_angle(vector3(0,0,1), rot_pos, pos_pos)
        return {'child_parent': self.joint_obj.parent,
                'child_parent_tf': child_parent_tf,
                'child_pose': dot(parent_pose, child_parent_tf),
                'connection': self.joint_obj}, \
               {'{}'.format(r_wheel_vel): Constraint(-wheel_vel_limit, 
                                                      wheel_vel_limit, r_wheel_vel),
                '{}'.format(l_wheel_vel): Constraint(-wheel_vel_limit, 
                                                      wheel_vel_limit, l_wheel_vel)} 


def create_diff_drive_joint_with_symbols(parent_pose, child_pose, connection_path, wheel_radius, wheel_distance, wheel_vel_limit, var_prefix):
    return SetDiffDriveJoint(parent_pose, child_pose, connection_path,
                Position((var_prefix + ('localization_x',)).to_symbol()),
                Position((var_prefix + ('localization_y',)).to_symbol()),
                Position((var_prefix + ('localization_z',)).to_symbol()),
                Position((var_prefix + ('localization_a',)).to_symbol()),
                Velocity((var_prefix + ('l_wheel',)).to_symbol()),
                Velocity((var_prefix + ('r_wheel',)).to_symbol()),
                wheel_vel_limit, wheel_radius, wheel_distance)


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
                                'Omnibase Joint', 
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
        child_parent_tf = dot(translation3(x_pos, y_pos, 0), rotation3_axis_angle(rot_axis, a_pos))
        return {'child_parent': self.joint_obj.parent,
                'child_parent_tf': child_parent_tf,
                'child_pose': dot(parent_pose, child_parent_tf),
                'connection': self.joint_obj}, \
               {'{}_lin_vel'.format(self.conn_path): Constraint(-lin_vel_limit, lin_vel_limit, get_diff(norm(vector3(x_pos, y_pos, 0)))),
                '{}_ang_vel'.format(self.conn_path): Constraint(-ang_vel_limit, ang_vel_limit, get_diff(a_pos))}

def create_omnibase_joint_with_symbols(parent_pose, child_pose, connection_path, rot_axis, lin_vel_limit, ang_vel_limit, var_prefix):
    return SetOmnibaseJoint(parent_pose, child_pose, connection_path, rot_axis,
                Position((var_prefix + ('localization_x',)).to_symbol()),
                Position((var_prefix + ('localization_y',)).to_symbol()),
                Position((var_prefix + ('localization_a',)).to_symbol()),
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

        pos_measure = dot_product(vector3(0,0,1), normed_axis)
        limit_dot_space = cos(pos_limit)

        return {'child_parent': self.joint_obj.parent,
                'child_parent_tf': dot(connection_tf, child_parent_tf),
                'child_pose': dot(parent_pose, connection_tf, rot_matrix, child_pose),
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
