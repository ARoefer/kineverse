from kineverse.gradients.diff_logic       import create_symbol, TYPE_POSITION, TYPE_VELOCITY
from kineverse.gradients.gradient_math    import *
from kineverse.model.articulation_model   import Constraint
from kineverse.operations.operation       import Operation
from kineverse.operations.urdf_operations import KinematicJoint, CreateURDFFrameConnection



class DiffDriveJoint(KinematicJoint):
    def __init__(self, parent, child, x_pos, y_pos, z_pos, a_pos, r_wheel_vel, l_wheel_vel, wheel_radius, wheel_distance, limit_vel=None):
        super(DiffDriveJoint, self).__init__('roomba', parent, child)
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.z_pos = z_pos
        self.a_pos = a_pos
        self.r_wheel_vel = r_wheel_vel
        self.l_wheel_vel = l_wheel_vel
        self.wheel_radius = wheel_radius 
        self.wheel_distance = wheel_distance 
        self.limit_vel = limit_vel

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

    def __deepcopy__(self, memo):
        out = DiffDriveJoint(self.parent,
                             self.child,
                             self.x_pos,
                             self.y_pos,
                             self.z_pos,
                             self.a_pos,
                             self.r_wheel_vel,
                             self.l_wheel_vel,
                             self.wheel_radius,
                             self.wheel_distance,
                             self.limit_vel)
        memo[id(self)] = out
        return out


def create_diff_drive_joint_with_symbols(parent_path, child_path, wheel_radius, wheel_distance, wheel_vel_limit, var_prefix):
    return DiffDriveJoint(parent_pose, child_pose,
                Position((var_prefix + ('localization_x',)).to_symbol()),
                Position((var_prefix + ('localization_y',)).to_symbol()),
                Position((var_prefix + ('localization_z',)).to_symbol()),
                Position((var_prefix + ('localization_a',)).to_symbol()),
                Velocity((var_prefix + ('l_wheel',)).to_symbol()),
                Velocity((var_prefix + ('r_wheel',)).to_symbol()),
                wheel_vel_limit, wheel_radius, wheel_distance)


class OmnibaseJoint(KinematicJoint):
    def __init__(self, parent, child, x_pos, y_pos, a_pos, rot_axis=vector3(0,0,1), limit_lin_vel=None, limit_ang_vel=None):
        super(OmnibaseJoint, self).__init__('omni', parent, child)
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.a_pos = a_pos
        self.rot_axis = rot_axis
        self.limit_lin_vel = limit_lin_vel
        self.limit_ang_vel = limit_ang_vel

    def _json_data(self, json_dict):
        super(OmnibaseJoint, self)._json_data(json_dict)
        del json_dict['jtype']
        json_dict.update({'x_pos'   : self.x_pos,
                          'y_pos'   : self.y_pos,
                          'a_pos'   : self.a_pos})

    def __eq__(self, other):
        if isinstance(other, OmnibaseJoint):
            return super(OmnibaseJoint, self).__eq__(other) and \
                                  self.x_pos == other.x_pos and \
                                  self.y_pos == other.y_pos and \
                                  self.a_pos == other.a_pos and \
                                  eq_expr(self.rot_axis, other.rot_axis) and \
                                  eq_expr(self.limit_lin_vel, other.limit_lin_vel) and \
                                  eq_expr(self.limit_ang_vel, other.limit_ang_vel)
        return False

    def __deepcopy__(self, memo):
        out = OmnibaseJoint(self.parent, 
                            self.child, 
                            self.x_pos, 
                            self.y_pos, 
                            self.a_pos, 
                            self.rot_axis,
                            self.limit_lin_vel, 
                            self.limit_ang_vel)
        memo[id(self)] = out
        return out


def create_omnibase_joint_with_symbols(parent_path, child_path, rot_axis, lin_vel_limit, ang_vel_limit, var_prefix):
    return OmnibaseJoint(parent_path, child_path,
                Position((var_prefix + ('localization_x',)).to_symbol()),
                Position((var_prefix + ('localization_y',)).to_symbol()),
                Position((var_prefix + ('localization_a',)).to_symbol()),
                rot_axis,
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

class CreateAdvancedFrameConnection(CreateURDFFrameConnection):

    def _execute_impl(self, joint_name, joint, parent_frame, child_frame):
        if joint.type == 'roomba':
            if joint.limit_vel is not None:
                self.constraints = {'{}'.format(joint.r_wheel_vel): Constraint(-joint.limit_vel, 
                                                                                joint.limit_vel, joint.r_wheel_vel),
                                    '{}'.format(joint.l_wheel_vel): Constraint(-joint.limit_vel, 
                                                                                joint.limit_vel, joint.l_wheel_vel)}
            else:
                self.constraints = {}

            pos_pos = point3(GC(x_pos, {r_wheel_vel: cos(a_pos) * wheel_radius * 0.5,
                                    l_wheel_vel: cos(a_pos) * wheel_radius * 0.5}), 
                             GC(y_pos, {r_wheel_vel: sin(a_pos) * wheel_radius * 0.5,
                                        l_wheel_vel: sin(a_pos) * wheel_radius * 0.5}), z_pos)

            rot_pos = GC(a_pos, {r_wheel_vel:   wheel_radius / wheel_distance,
                                 l_wheel_vel: - wheel_radius / wheel_distance})

            self.child_parent        = joint.parent
            self.child_relative_pose = frame3_axis_angle(vector3(0,0,1), rot_pos, pos_pos)
            self.child_full_pose     = dot(parent_frame.pose, self.child_relative_pose)
            self.child_link_joint    = joint_name
        elif joint.type == 'omni':
            self.constraints = {}
            if joint.limit_lin_vel is not None:
                self.constraints['{}_lin_velocity'.format(joint_name)] = Constraint(-joint.limit_lin_vel,
                                                                                    joint.limit_lin_vel,
                                                                                   get_diff(norm(vector3(joint.x_pos, joint.y_pos, 0))))
            if joint.limit_ang_vel is not None:
                self.constraints['{}_ang_velocity'.format(joint_name)] = Constraint(-joint.limit_ang_vel,
                                                                                    joint.limit_ang_vel,
                                                                                    get_diff(joint.a_pos))

            self.child_parent        = joint.parent
            self.child_relative_pose = dot(translation3(joint.x_pos, joint.y_pos, 0),
                                           rotation3_axis_angle(joint.rot_axis, joint.a_pos))
            self.child_full_pose     = dot(parent_frame.pose, self.child_relative_pose)  
            self.child_link_joint    = joint_name
        elif joint.type == 'ball':
            raise NotImplementedError
        elif joint.type == '2dof':
            raise NotImplementedError
        else:
            super(CreateAdvancedFrameConnection, self)._execute_impl(joint_name, joint, parent_frame, child_frame)

# class Set2DofRotationJoint(Operation):
#     def init(self, parent_pose, child_pose, connection_path, connection_tf, axis_x, axis_y, pos)
