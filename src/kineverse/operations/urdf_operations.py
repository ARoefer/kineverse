import urdf_parser_py.urdf as urdf

import kineverse.gradients.common_math as cm
from kineverse.gradients.diff_logic import Position
from kineverse.gradients.gradient_math import get_diff, \
    matrix_wrapper, \
    rotation3_rpy, \
    rotation3_axis_angle, \
    translation3, \
    vector3, \
    dot
from kineverse.model.articulation_model import Constraint
from kineverse.model.geometry_model import RigidBody, \
    KinematicJoint, \
    Geometry, \
    InertialData, \
    ArticulatedObject, \
    GEOM_TYPE_MESH, \
    GEOM_TYPE_BOX, \
    GEOM_TYPE_CYLINDER, \
    GEOM_TYPE_SPHERE
from kineverse.operations.basic_operations import Operation, \
    CreateValue, \
    ExecFunction, \
    Path  # ,       \


def urdf_origin_to_transform(origin):
    if origin is not None:
        xyz = origin.xyz if origin.xyz is not None else [0.0, 0.0, 0.0]
        rpy = origin.rpy if origin.rpy is not None else [0.0, 0.0, 0.0]
        return dot(translation3(*xyz), rotation3_rpy(*rpy))
    return cm.eye(4)


def urdf_axis_to_vector(axis):
    return vector3(*axis) if axis is not None else vector3(1.0, 0.0, 0.0)


class Kinematic1DofJoint(KinematicJoint):
    def __init__(self, jtype, parent, child, position):
        super(Kinematic1DofJoint, self).__init__(jtype, parent, child)
        self.position = position

    def _json_data(self, json_dict):
        super(Kinematic1DofJoint, self)._json_data(json_dict)
        json_dict.update({'position': self.position})

    def __eq__(self, other):
        if isinstance(other, Kinematic1DofJoint):
            return super(Kinematic1DofJoint, self).__eq__(other) and self.position == other.position
        return False


class FixedJoint(KinematicJoint):
    def __init__(self, parent, child, tf_offset=cm.eye(4)):
        super(FixedJoint, self).__init__('fixed', parent, child)
        self.tf_offset = tf_offset

    def _json_data(self, json_dict):
        super(FixedJoint, self)._json_data(json_dict)
        del json_dict['jtype']

    def __deepcopy__(self, memo):
        out = type(self)(self.parent, self.child, self.tf_offset * 1)
        memo[id(self)] = out
        return out


class PrismaticJoint(KinematicJoint):
    def __init__(self, parent, child, position, axis, tf_offset=cm.eye(4), limit_lower=-1e9, limit_upper=1e9,
                 limit_vel=1e9, multiplier=None, offset=None):
        super(PrismaticJoint, self).__init__('prismatic', parent, child)
        self.position = position
        self.axis = axis
        self.tf_offset = tf_offset
        self.limit_lower = limit_lower
        self.limit_upper = limit_upper
        self.limit_vel = limit_vel
        self.multiplier = multiplier
        self.offset = offset

    def _json_data(self, json_dict):
        super(PrismaticJoint, self)._json_data(json_dict)
        del json_dict['jtype']
        json_dict.update({'position': self.position})

    def __deepcopy__(self, memo):
        out = type(self)(self.parent,
                         self.child,
                         self.position,
                         self.axis * 1,
                         self.tf_offset * 1,
                         self.limit_lower,
                         self.limit_upper,
                         self.limit_vel,
                         self.multiplier,
                         self.offset)
        memo[id(self)] = out
        return out


class ContinuousJoint(KinematicJoint):
    def __init__(self, parent, child, position, axis, tf_offset=cm.eye(4), limit_vel=1e9, multiplier=None, offset=None):
        super(ContinuousJoint, self).__init__('continuous', parent, child)
        self.position = position
        self.axis = axis
        self.tf_offset = tf_offset
        self.limit_vel = limit_vel
        self.multiplier = multiplier
        self.offset = offset

    def _json_data(self, json_dict):
        super(ContinuousJoint, self)._json_data(json_dict)
        del json_dict['jtype']
        json_dict.update({'position': self.position})

    def __deepcopy__(self, memo):
        out = type(self)(self.parent,
                         self.child,
                         self.position,
                         self.axis * 1,
                         self.tf_offset * 1,
                         self.limit_vel,
                         self.multiplier,
                         self.offset)
        memo[id(self)] = out
        return out


class RevoluteJoint(KinematicJoint):
    def __init__(self, parent, child, position, axis, tf_offset=cm.eye(4), limit_lower=-1e9, limit_upper=1e9,
                 limit_vel=1e9, multiplier=None, offset=None):
        super(RevoluteJoint, self).__init__('revolute', parent, child)
        self.position = position
        self.axis = axis
        self.tf_offset = tf_offset
        self.limit_lower = limit_lower
        self.limit_upper = limit_upper
        self.limit_vel = limit_vel
        self.multiplier = multiplier
        self.offset = offset

    def _json_data(self, json_dict):
        super(RevoluteJoint, self)._json_data(json_dict)
        del json_dict['jtype']
        json_dict.update({'position': self.position})

    def __deepcopy__(self, memo):
        out = type(self)(self.parent,
                         self.child,
                         self.position,
                         self.axis * 1,
                         self.tf_offset * 1,
                         self.limit_lower,
                         self.limit_upper,
                         self.limit_vel,
                         self.multiplier,
                         self.offset)
        memo[id(self)] = out
        return out


class CreateURDFFrameConnection(Operation):
    def __init__(self, joint, parent_frame, child_frame):
        super(CreateURDFFrameConnection, self).__init__(
            {'child_parent': child_frame + ('parent',),
             'child_relative_pose': child_frame + ('to_parent',),
             'child_full_pose': child_frame + ('pose',)},
            joint_name=str(joint), joint=joint, parent_frame=parent_frame, child_frame=child_frame)

    def _execute_impl(self, joint_name, joint, parent_frame, child_frame):
        constraints = {}
        if joint.type == 'fixed':
            new_local_fk = dot(joint.tf_offset, child_frame.to_parent)
        else:
            vel = get_diff(joint.position)
            pos = joint.position
            if joint.multiplier is not None:
                pos *= joint.multiplier
            if joint.offset is not None:
                pos += joint.offset

            if hasattr(joint, 'limit_lower') and joint.limit_lower is not None:
                constraints['{}_position'.format(joint_name)] = Constraint(joint.limit_lower - pos,
                                                                           joint.limit_upper - pos,
                                                                           pos)
            if joint.limit_vel is not None:
                constraints['{}_velocity'.format(joint_name)] = Constraint(-joint.limit_vel,
                                                                           joint.limit_vel,
                                                                           vel)

            if joint.type == 'prismatic':
                new_local_fk = dot(joint.tf_offset,
                                   translation3(joint.axis[0] * pos,
                                                joint.axis[1] * pos,
                                                joint.axis[2] * pos),
                                   child_frame.to_parent)
            elif joint.type == 'continuous' or joint.type == 'revolute':
                new_local_fk = dot(joint.tf_offset,
                                   rotation3_axis_angle(joint.axis, pos),
                                   child_frame.to_parent)
            else:
                raise Exception('Unknown joint type "{}". Failed to instantiate connection.'.format(joint.type))

        self.child_parent = joint.parent
        self.child_relative_pose = new_local_fk
        self.child_full_pose = dot(parent_frame.pose, new_local_fk)
        self.constraints = constraints


urdf_geom_types = {urdf.Mesh: GEOM_TYPE_MESH,
                   urdf.Box: GEOM_TYPE_BOX,
                   urdf.Cylinder: GEOM_TYPE_CYLINDER,
                   urdf.Sphere: GEOM_TYPE_SPHERE}


def urdf_to_geometry(urdf_geom, to_modify):
    to_modify.type = urdf_geom_types[type(urdf_geom)]
    if to_modify.type == 'mesh':
        to_modify.mesh = urdf_geom.filename
        if urdf_geom.scale is not None:
            to_modify.scale = vector3(*urdf_geom.scale)
    elif to_modify.type == 'box':
        to_modify.scale = vector3(*urdf_geom.size)
    elif to_modify.type == 'cylinder':
        to_modify.scale = vector3(urdf_geom.radius * 2, urdf_geom.radius * 2, urdf_geom.length)
    elif to_modify.type == 'sphere':
        to_modify.scale = vector3(urdf_geom.radius * 2, urdf_geom.radius * 2, urdf_geom.radius * 2)
    else:
        raise Exception('Can not convert geometry of type "{}"'.format(to_modify.type))


def load_urdf(ks,
              prefix,
              urdf,
              reference_frame='world',
              joint_prefix=None,
              limit_prefix=None,
              limit_symbols=False,
              robot_class=ArticulatedObject):
    """

    :param ks:
    :type urdf: urdf_parser_py.urdf.Robot
    :type prefix: str
    :param reference_frame:
    :param joint_prefix:
    :param robot_class:
    :return:
    """
    if not issubclass(robot_class, ArticulatedObject):
        raise Exception(
            'Robot class needs to be a subclass of {} but is given class {} is not'.format(ArticulatedObject,
                                                                                           robot_class))

    if type(prefix) == str:
        prefix = Path(prefix)

    if joint_prefix is None:
        joint_prefix = prefix

    if type(joint_prefix) == str:
        joint_prefix = Path(joint_prefix)

    if limit_prefix is None:
        limit_prefix = prefix

    if isinstance(limit_prefix, str):
        limit_prefix = Path(limit_prefix)

    ks.apply_operation('create {}'.format(str(prefix)), CreateValue(prefix, robot_class(urdf.name)))

    for u_link in urdf.links:
        link_path = prefix + Path(['links', u_link.name])
        collision = None
        geometry = None
        inertial = InertialData(link_path, cm.eye(4))
        if hasattr(u_link, 'collisions') and u_link.collisions is not None and len(u_link.collisions) > 0:
            collision = {}
            for x, c in enumerate(u_link.collisions):
                collision[x] = Geometry(str(link_path), urdf_origin_to_transform(c.origin), '')
                urdf_to_geometry(c.geometry, collision[x])
        elif u_link.collision is not None:
            collision = {'0': Geometry(str(link_path), urdf_origin_to_transform(u_link.collision.origin), '')}
            urdf_to_geometry(u_link.collision.geometry, collision.values()[0])

        if hasattr(u_link, 'visuals') and u_link.visuals is not None and len(u_link.visuals) > 0:
            geometry = {}
            for x, v in enumerate(u_link.visuals):
                geometry[x] = Geometry(str(link_path), urdf_origin_to_transform(v.origin), '')
                urdf_to_geometry(v.geometry, geometry[x])
        elif u_link.visual is not None:
            geometry = {'0': Geometry(str(link_path), urdf_origin_to_transform(u_link.visual.origin), '')}
            urdf_to_geometry(u_link.visual.geometry, geometry.values()[0])

        if u_link.inertial is not None:
            if u_link.inertial.origin is not None:
                inertial.pose = urdf_origin_to_transform(u_link.inertial.origin)
                inertial.to_parent = inertial.pose
            inertial.mass = u_link.inertial.mass
            uin = u_link.inertial.inertia  # Just for readability
            inertial.inertia_matrix = matrix_wrapper([[uin.ixx, uin.ixy, uin.ixz],
                                                      [uin.ixy, uin.iyy, uin.iyz],
                                                      [uin.ixz, uin.iyz, uin.izz]])

        ks.apply_operation('create {}'.format(str(prefix + Path(u_link.name))),
                           CreateValue(link_path,
                                       RigidBody(reference_frame, urdf_origin_to_transform(u_link.origin),
                                                 None, geometry, collision, inertial)))

    limit_map = {}

    if urdf.get_root() in urdf.child_map:
        joints_left = urdf.child_map[urdf.get_root()]

        joint_set = set()

        connection_order = []

        while len(joints_left) > 0:
            n_joint, n_link = joints_left[0]
            joints_left = joints_left[1:]

            if n_joint not in connection_order:
                connection_order.append(n_joint)

            # Create transformation operation modeling joint
            u_joint = urdf.joint_map[n_joint]
            if u_joint.mimic is not None:
                if u_joint.mimic.joint in joint_set:
                    multiplier = u_joint.mimic.multiplier
                    offset = u_joint.mimic.offset
                    position = prefix + ('joints', u_joint.mimic.joint, 'position')
                else:
                    if n_link in urdf.child_map:
                        joints_left += [(j, l) for j, l in urdf.child_map[n_link] if j not in joint_set]
                    joints_left.append((n_joint, n_link))
                    continue
            else:
                position = Position((joint_prefix + (u_joint.name,)).to_symbol())
                multiplier = None
                offset = None

            lower_limit_symbol = None
            upper_limit_symbol = None
            vel_limit = None
            if u_joint.limit is not None:
                vel_limit = (limit_prefix + (
                u_joint.name, 'velocity')).to_symbol() if limit_symbols else u_joint.limit.velocity

                limit_map[u_joint.name] = {'velocity': u_joint.limit.velocity}
                if u_joint.limit.lower is not None and u_joint.type != 'continuous':
                    try:
                        lower_limit = max(u_joint.safety_controller.soft_lower_limit, u_joint.limit.lower)
                        upper_limit = min(u_joint.safety_controller.soft_upper_limit, u_joint.limit.upper)
                    except AttributeError:
                        lower_limit = u_joint.limit.lower
                        upper_limit = u_joint.limit.upper
                    lower_limit_symbol = (limit_prefix + (
                    u_joint.name, 'position', 'lower')).to_symbol() if limit_symbols else lower_limit
                    upper_limit_symbol = (limit_prefix + (
                    u_joint.name, 'position', 'upper')).to_symbol() if limit_symbols else upper_limit
                    limit_map[u_joint.name]['position'] = {'lower': lower_limit,
                                                           'upper': upper_limit}
                else:
                    pass

            parent_path = prefix + ('links', u_joint.parent)
            child_path = prefix + ('links', u_joint.child)

            if u_joint.type == 'fixed':
                joint_op = ExecFunction(prefix + ('joints', u_joint.name),
                                        FixedJoint,
                                        str(parent_path),
                                        str(child_path),
                                        urdf_origin_to_transform(u_joint.origin))
            elif u_joint.type == 'prismatic':
                joint_op = ExecFunction(prefix + ('joints', u_joint.name),
                                        PrismaticJoint,
                                        str(parent_path),
                                        str(child_path),
                                        position,
                                        urdf_axis_to_vector(u_joint.axis),
                                        urdf_origin_to_transform(u_joint.origin),
                                        lower_limit_symbol,
                                        upper_limit_symbol,
                                        vel_limit,
                                        multiplier,
                                        offset)
            elif u_joint.type == 'continuous':
                joint_op = ExecFunction(prefix + ('joints', u_joint.name),
                                        ContinuousJoint,
                                        str(parent_path),
                                        str(child_path),
                                        position,
                                        urdf_axis_to_vector(u_joint.axis),
                                        urdf_origin_to_transform(u_joint.origin),
                                        vel_limit,
                                        multiplier,
                                        offset)
            elif u_joint.type == 'revolute':
                joint_op = ExecFunction(prefix + ('joints', u_joint.name),
                                        RevoluteJoint,
                                        str(parent_path),
                                        str(child_path),
                                        position,
                                        urdf_axis_to_vector(u_joint.axis),
                                        urdf_origin_to_transform(u_joint.origin),
                                        lower_limit_symbol,
                                        upper_limit_symbol,
                                        vel_limit,
                                        multiplier,
                                        offset)
            else:
                raise Exception('Joint type "{}" is currently not covered.'.format(u_joint.type))
            ks.apply_operation('create {}'.format(str(prefix + ('joints', u_joint.name))), joint_op)

            joint_set.add(n_joint)
            if n_link in urdf.child_map:
                joints_left += [(j, l) for j, l in urdf.child_map[n_link] if j not in joint_set]

        for n_joint in connection_order:
            joint_path = prefix + ('joints', n_joint)
            joint = ks.get_data(joint_path)

            ks.apply_operation(
                'connect {} {}'.format(joint.parent, joint.child),
                CreateURDFFrameConnection(joint_path, Path(joint.parent), Path(joint.child)))

    return limit_map
