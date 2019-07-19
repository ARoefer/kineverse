from kineverse.gradients.gradient_math     import frame3_rpy,           \
                                                  get_diff,             \
                                                  matrix_wrapper,       \
                                                  rotation3_rpy,        \
                                                  rotation3_axis_angle, \
                                                  spw,                  \
                                                  translation3,         \
                                                  vector3
from kineverse.gradients.diff_logic        import create_symbol, TYPE_POSITION
from kineverse.operations.basic_operations import Operation,           \
                                                  CreateComplexObject, \
                                                  Path,                \
                                                  collect_paths
from kineverse.model.kinematic_model       import Constraint
from kineverse.type_sets                   import matrix_types

def urdf_origin_to_transform(origin):
    if origin is not None:
        xyz = origin.xyz if origin.xyz is not None else [0,0,0]
        rpy = origin.rpy if origin.rpy is not None else [0,0,0]
        return translation3(*xyz) * rotation3_rpy(*rpy)
    return spw.eye(4)

def urdf_axis_to_vector(axis):
    return vector3(*axis) if axis is not None else vector3(1,0,0)

class URDFInertial(object):
    def __init__(self, origin, mass, ixx, ixy, ixz, iyy, iyz, izz):
        self.pose   = origin if type(origin) in matrix_types else frame3_rpy(origin.rpy[0], origin.p[1], origin.rpy[2], origin.xyz)
        self.mass   = mass
        self.tensor = matrix_wrapper([[ixx, ixy, ixz], 
                                      [ixy, iyy, iyz],
                                      [ixz, iyz, izz]])

class URDFGeometry(object):
    def __init__(self, shape, size):
        self.shape = shape
        self.size  = size if type(size) in matrix_types else matrix_wrapper(*size)

class URDFVisual(object):
    def __init__(self, origin, geometry):
        self.pose     = origin if type(origin) in matrix_types else frame3_rpy(origin.rpy[0], origin.p[1], origin.rpy[2], origin.xyz)
        self.geometry = URDFGeometry()

class URDFCollision(object):
    def __init__(self, origin, geometry):
        self.pose     = origin if type(origin) in matrix_types else frame3_rpy(origin.rpy[0], origin.p[1], origin.rpy[2], origin.xyz)
        self.geometry = URDFGeometry()        

class URDFLink(object):
    def __init__(self, ):
        self.inertial   = URDFInertial()
        self.visuals    = {URDFVisual()}
        self.collisions = {URDFCollision()}


class URDFLimit(object):
    def __init__(self, lower, upper, effort, velocity):
        self.lower    = lower
        self.upper    = upper
        self.effort   = effort
        self.velocity = velocity

class URDFJoint(object):
    def __init__(self, ):
        self.pose   = origin if type(origin) in matrix_types else frame3_rpy(origin.rpy[0], origin.p[1], origin.rpy[2], origin.xyz)
        self.parent = parent_str
        self.child  = child_str
        self.axis   = axis if type(axis) in matrix_types else vector3(*origin.xyz)
        # self.calibration = URDFCalibration() Not for now, too control specific
        self.damping  = damping
        self.friction = friction
        self.limit    = URDFLimit()
        self.mimic    = mimic_term


class KinematicJoint(object):
    def __init__(self, jtype, parent, child):
        self.type     = jtype
        self.parent   = parent
        self.child    = child

class Kinematic1DofJoint(KinematicJoint):
    def __init__(self, jtype, parent, child, position):
        super(Kinematic1DofJoint, self).__init__(jtype, parent, child)
        self.position = position

class FixedJoint(KinematicJoint):
    def __init__(self, parent, child):
        super(FixedJoint, self).__init__('fixed', parent, child)   

class PrismaticJoint(KinematicJoint):
    def __init__(self, parent, child, position):
        super(PrismaticJoint, self).__init__('prismatic', parent, child)   
        self.position = position

class ContinuousJoint(KinematicJoint):
    def __init__(self, parent, child, position):
        super(ContinuousJoint, self).__init__('continuous', parent, child)   
        self.position = position

class RevoluteJoint(KinematicJoint):
    def __init__(self, parent, child, position):
        super(RevoluteJoint, self).__init__('revolute', parent, child)   
        self.position = position

# class PrismaticJoint(Kinematic1DofJoint):
#     def __init__(self, parent, child, axis, position, lower_limit=-1e9, upper_limit=1e9):
#         super(PrismaticJoint, self).__init__('prismatic', parent, child, axis, position)
#         self.lower_limit = lower_limit
#         self.upper_limit = upper_limit
        
# class RevoluteJoint(Kinematic1DofJoint):
#     def __init__(self, parent, child, axis, position):
#         super(RevoluteJoint, self).__init__('revolute', parent, child, axis, position)

# class HingeJoint(Kinematic1DofJoint):
#     def __init__(self, parent, child, axis, position, lower_limit=-1e9, upper_limit=1e9):
#         super(HingeJoint, self).__init__('hinge', parent, child, axis, position)
#         self.lower_limit = lower_limit
#         self.upper_limit = upper_limit

class KinematicLink(object):
    def __init__(self, pose):
        self.pose      = pose
        self.geometry  = None
        self.collision = None

class URDFRobot(object):
    def __init__(self, name):
        self.name   = name
        self.links  = {}
        self.joints = {}

class SetConnection(Operation):
    def __init__(self, name, joint_obj, parent_pose, child_pose, connection_path, connection_tf):
        self.joint_obj = joint_obj
        attrs = collect_paths(self.joint_obj, Path('connection'))
        super(SetConnection, self).__init__(name, 
                                            ['child_pose'] + [str(a) for a in attrs], 
                                            parent_pose=parent_pose, 
                                            child_pose=child_pose,  
                                            connection_tf=connection_tf,
                                            **{str(a): connection_path + a[1:] for a in attrs})

    def _apply(self, ks, parent_pose, child_pose, connection_tf):
        return {'child_pose': parent_pose * connection_tf * child_pose,
                'connection': self.joint_obj}, {}

class SetFixedJoint(SetConnection):
    def __init__(self, parent_pose, child_pose, connection_path, connection_tf):
        super(SetFixedJoint, self).__init__('Fixed Joint', 
                                            FixedJoint(str(parent_pose[:-1]), 
                                                       str(child_pose[:-1])),
                                            parent_pose, child_pose, connection_path, connection_tf)

class SetPrismaticJoint(Operation):
    def __init__(self, parent_pose, child_pose, connection_path, connection_tf, axis, position, lower_limit, upper_limit, vel_limit, mimic_m=None, mimic_o=None):
        self.joint_obj = PrismaticJoint(str(parent_pose[:-1]), str(child_pose[:-1]), position)
        self.conn_path = connection_path
        attrs = collect_paths(self.joint_obj, Path('connection'))
        super(SetPrismaticJoint, self).__init__('Prismatic Joint', 
                                            ['child_pose'] + [str(a) for a in attrs], 
                                            parent_pose=parent_pose, 
                                            child_pose=child_pose, 
                                            connection_tf=connection_tf,
                                            axis=axis,
                                            position=position,
                                            lower_limit=lower_limit,
                                            upper_limit=upper_limit,
                                            vel_limit=vel_limit,
                                            mimic_m=mimic_m,
                                            mimic_o=mimic_o,
                                            **{str(a): connection_path + a[1:] for a in attrs})

    def _apply(self, ks, parent_pose, child_pose, connection_tf, axis, position, lower_limit, upper_limit, vel_limit, mimic_m, mimic_o):
        return {'child_pose': parent_pose * connection_tf * translation3(*(axis[:,:3] * position)) * child_pose,
                'connection': self.joint_obj}, \
               {'{}_position'.format(self.conn_path): Constraint(lower_limit, upper_limit, get_diff(position)),
                '{}_velocity'.format(self.conn_path): Constraint(-vel_limit, vel_limit, get_diff(position))}

class SetRevoluteJoint(Operation):
    def __init__(self, parent_pose, child_pose, connection_path, connection_tf, axis, position, lower_limit, upper_limit, vel_limit, mimic_m=None, mimic_o=None):
        self.joint_obj = RevoluteJoint(str(parent_pose[:-1]), str(child_pose[:-1]), position)
        self.conn_path = connection_path
        attrs = collect_paths(self.joint_obj, Path('connection'))
        super(SetRevoluteJoint, self).__init__('Revolute Joint', 
                                            ['child_pose'] + [str(a) for a in attrs], 
                                            parent_pose=parent_pose, 
                                            child_pose=child_pose, 
                                            connection_tf=connection_tf,
                                            axis=axis,
                                            position=position,
                                            lower_limit=lower_limit,
                                            upper_limit=upper_limit,
                                            vel_limit=vel_limit,
                                            mimic_m=mimic_m,
                                            mimic_o=mimic_o,
                                            **{str(a): connection_path + a[1:] for a in attrs})

    def _apply(self, ks, parent_pose, child_pose, connection_tf, axis, position, lower_limit, upper_limit, vel_limit, mimic_m, mimic_o):
        position = position if mimic_m is None or mimic_o is None else position * mimic_m + mimic_o
        return {'child_pose': parent_pose * connection_tf * rotation3_axis_angle(axis, position) * child_pose,
                'connection': self.joint_obj}, \
               {'{}_position'.format(self.conn_path): Constraint(lower_limit, upper_limit, get_diff(position)),
               '{}_velocity'.format(self.conn_path): Constraint(-vel_limit, vel_limit, get_diff(position))}

class SetContinuousJoint(Operation):
    def __init__(self, parent_pose, child_pose, connection_path, connection_tf, axis, position,vel_limit, mimic_m=None, mimic_o=None):
        self.joint_obj = ContinuousJoint(str(parent_pose[:-1]), str(child_pose[:-1]), position)
        self.conn_path = connection_path
        attrs = collect_paths(self.joint_obj, Path('connection'))
        super(SetContinuousJoint, self).__init__('Continuous Joint', 
                                            ['child_pose'] + [str(a) for a in attrs], 
                                            parent_pose=parent_pose, 
                                            child_pose=child_pose, 
                                            connection_tf=connection_tf,
                                            axis=axis,
                                            position=position,
                                            vel_limit=vel_limit,
                                            mimic_m=mimic_m,
                                            mimic_o=mimic_o,
                                            **{str(a): connection_path + a[1:] for a in attrs}) 

    def _apply(self, ks, parent_pose, child_pose, connection_tf, axis, position, vel_limit, mimic_m, mimic_o):
        position = position if mimic_m is None or mimic_o is None else position * mimic_m + mimic_o
        return {'child_pose': parent_pose * connection_tf * rotation3_axis_angle(axis, position) * child_pose,
                'connection': self.joint_obj}, \
               {'{}_velocity'.format(self.conn_path): Constraint(-vel_limit, vel_limit, get_diff(position))}


def load_urdf(ks, prefix, urdf):
    ks.apply_operation(CreateComplexObject(prefix, URDFRobot(urdf.name)), 'create_{}'.format(str(prefix)))

    for u_link in urdf.links:
        ks.apply_operation(CreateComplexObject(prefix + Path(['links', u_link.name]), 
                                               KinematicLink(urdf_origin_to_transform(u_link.origin))),
                                               'create_{}'.format(str(prefix + Path(u_link.name))))

    links_left = [urdf.get_root()]
    joint_set  = set()

    while len(links_left) > 0:
        n_plink    = links_left[0]
        links_left = links_left[1:]
        u_children = urdf.child_map[n_plink]

        for n_joint, n_link in u_children:

            # Create transformation operation modeling joint
            u_joint = urdf.joint_map[n_joint]
            if u_joint.mimic is not None:
                multiplier = u_joint.mimic.multiplier 
                offset     = u_joint.mimic.offset
                position   = prefix + ('joints', u_joint.mimic.joint, 'position')
            else:
                position   = create_symbol((prefix + (u_joint.name, )).to_symbol(), TYPE_POSITION)
                multiplier = None
                offset     = None

            lower_limit = -1e9
            upper_limit =  1e9
            vel_limit   =  1e9
            if u_joint.limit is not None:
                lower_limit = u_joint.limit.lower if u_joint.limit.lower is not None else -1e9
                upper_limit = u_joint.limit.upper if u_joint.limit.upper is not None else  1e9
                vel_limit   = u_joint.limit.velocity


            if u_joint.type == 'fixed':
                op = SetFixedJoint(prefix + ('links', u_joint.parent, 'pose'),
                                   prefix + ('links', u_joint.child,  'pose'),
                                   prefix + ('joints', u_joint.name),
                                   urdf_origin_to_transform(u_joint.origin))
            elif u_joint.type == 'prismatic':
                op = SetPrismaticJoint(prefix + ('links', u_joint.parent, 'pose'),
                                       prefix + ('links', u_joint.child,  'pose'),
                                       prefix + ('joints', u_joint.name),
                                       urdf_origin_to_transform(u_joint.origin),
                                       urdf_axis_to_vector(u_joint.axis),
                                       position,
                                       lower_limit, 
                                       upper_limit,
                                       vel_limit,
                                       multiplier,
                                       offset)
            elif u_joint.type == 'continuous':
                op = SetContinuousJoint(prefix + ('links', u_joint.parent, 'pose'),
                                       prefix + ('links', u_joint.child,  'pose'),
                                       prefix + ('joints', u_joint.name),
                                       urdf_origin_to_transform(u_joint.origin),
                                       urdf_axis_to_vector(u_joint.axis),
                                       position,
                                       vel_limit,
                                       multiplier,
                                       offset)
            elif u_joint.type == 'revolute':
                op = SetRevoluteJoint(prefix + ('links', u_joint.parent, 'pose'),
                                       prefix + ('links', u_joint.child,  'pose'),
                                       prefix + ('joints', u_joint.name),
                                       urdf_origin_to_transform(u_joint.origin),
                                       urdf_axis_to_vector(u_joint.axis),
                                       position,
                                       lower_limit, 
                                       upper_limit,
                                       vel_limit,
                                       multiplier,
                                       offset)
            else:
                raise Exception('Joint type "{}" is currently not covered.'.format(u_joint.type))
            ks.apply_operation(op, 'connect {} {}'.format(str(prefix + ('links', u_joint.parent)), str(prefix + ('links', u_joint.child)))) 

            joint_set.add(n_joint)
            if n_link in urdf.child_map:
                links_left.append(n_link)