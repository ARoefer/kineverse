import urdf_parser_py.urdf as urdf

from kineverse.gradients.gradient_math     import frame3_rpy,           \
                                                  get_diff,             \
                                                  matrix_wrapper,       \
                                                  rotation3_rpy,        \
                                                  rotation3_axis_angle, \
                                                  spw,                  \
                                                  translation3,         \
                                                  vector3
from kineverse.gradients.diff_logic        import create_pos
from kineverse.operations.basic_operations import Operation,           \
                                                  CreateComplexObject, \
                                                  Path,                \
                                                  collect_paths#,       \
from kineverse.operations.operation        import op_construction_wrapper
from kineverse.model.kinematic_model       import Constraint
from kineverse.model.geometry_model        import RigidBody, \
                                                  KinematicJoint, \
                                                  Geometry, \
                                                  InertialData, \
                                                  ArticulatedObject
from kineverse.type_sets                   import matrix_types
from kineverse.utils                       import deepcopy


def urdf_origin_to_transform(origin):
    if origin is not None:
        xyz = origin.xyz if origin.xyz is not None else [0,0,0]
        rpy = origin.rpy if origin.rpy is not None else [0,0,0]
        return translation3(*xyz) * rotation3_rpy(*rpy)
    return spw.eye(4)

def urdf_axis_to_vector(axis):
    return vector3(*axis) if axis is not None else vector3(1,0,0)



class Kinematic1DofJoint(KinematicJoint):
    def __init__(self, jtype, parent, child, position):
        super(Kinematic1DofJoint, self).__init__(jtype, parent, child)
        self.position = position

    def _json_data(self, json_dict):
        super(Kinematic1DofJoint, self)._json_data(json_dict)
        json_dict.update({'position': self.position})


class FixedJoint(KinematicJoint):
    def __init__(self, parent, child):
        super(FixedJoint, self).__init__('fixed', parent, child)   

    def _json_data(self, json_dict):
        super(FixedJoint, self)._json_data(json_dict)
        del json_dict['jtype']

    def __deepcopy__(self, memo):
        out = FixedJoint(self.parent, self.child)
        memo[id(self)] = out
        return out

class PrismaticJoint(KinematicJoint):
    def __init__(self, parent, child, position):
        super(PrismaticJoint, self).__init__('prismatic', parent, child)   
        self.position = position

    def _json_data(self, json_dict):
        super(PrismaticJoint, self)._json_data(json_dict)
        del json_dict['jtype']
        json_dict.update({'position': self.position})

    def __deepcopy__(self, memo):
        out = PrismaticJoint(self.parent, self.child, self.position)
        memo[id(self)] = out
        return out

class ContinuousJoint(KinematicJoint):
    def __init__(self, parent, child, position):
        super(ContinuousJoint, self).__init__('continuous', parent, child)   
        self.position = position

    def _json_data(self, json_dict):
        super(ContinuousJoint, self)._json_data(json_dict)
        del json_dict['jtype']
        json_dict.update({'position': self.position})

    def __deepcopy__(self, memo):
        out = ContinuousJoint(self.parent, self.child, self.position)
        memo[id(self)] = out
        return out

class RevoluteJoint(KinematicJoint):
    def __init__(self, parent, child, position):
        super(RevoluteJoint, self).__init__('revolute', parent, child)   
        self.position = position

    def _json_data(self, json_dict):
        super(RevoluteJoint, self)._json_data(json_dict)
        del json_dict['jtype']
        json_dict.update({'position': self.position})

    def __deepcopy__(self, memo):
        out = RevoluteJoint(self.parent, self.child, self.position)
        memo[id(self)] = out
        return out

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


class KinematicLink(RigidBody):
    pass

URDFRobot = ArticulatedObject


class SetConnection(Operation):
    def init(self, name, joint_obj, parent_pose, child_pose, connection_path, connection_tf):
        self.joint_obj = joint_obj
        op_construction_wrapper(super(SetConnection, self).init,
                                name, 
                                ['child_pose', 'child_parent', 'child_parent_tf'],
                                (connection_path, 'connection', self.joint_obj),
                                parent_pose=parent_pose,
                                child_parent=child_pose[:-1] + ('parent',),
                                child_parent_tf=child_pose[:-1] + ('to_parent',),
                                child_pose=child_pose,  
                                connection_tf=connection_tf)

    def _apply(self, ks, parent_pose, child_pose, child_parent_tf, connection_tf):
        return {'child_parent': self.joint_obj.parent,
                'child_parent_tf': connection_tf * child_parent_tf,
                'child_pose': parent_pose * connection_tf * child_pose,
                'connection': self.joint_obj}, {}

class SetFixedJoint(SetConnection):
    def init(self, parent_pose, child_pose, connection_path, connection_tf):
        super(SetFixedJoint, self).init('Fixed Joint', 
                                        FixedJoint(str(parent_pose[:-1]), 
                                                   str(child_pose[:-1])),
                                            parent_pose, child_pose, connection_path, connection_tf)

class SetPrismaticJoint(Operation):
    def init(self, parent_pose, child_pose, connection_path, connection_tf, axis, position, lower_limit, upper_limit, vel_limit, mimic_m=None, mimic_o=None):
        self.joint_obj = PrismaticJoint(str(parent_pose[:-1]), str(child_pose[:-1]), position)
        self.conn_path = connection_path
        op_construction_wrapper(super(SetPrismaticJoint, self).init, 
                                'Prismatic Joint',            
                                ['child_pose', 'child_parent', 'child_parent_tf'], 
                                (connection_path, 'connection', self.joint_obj),
                                parent_pose=parent_pose,
                                child_parent=child_pose[:-1] + ('parent',),
                                child_parent_tf=child_pose[:-1] + ('to_parent',),
                                child_pose=child_pose, 
                                connection_tf=connection_tf,
                                axis=axis,
                                position=position,
                                lower_limit=lower_limit,
                                upper_limit=upper_limit,
                                vel_limit=vel_limit,
                                mimic_m=mimic_m,
                                mimic_o=mimic_o)

    def _apply(self, ks, parent_pose, child_pose, child_parent_tf, connection_tf, axis, position, lower_limit, upper_limit, vel_limit, mimic_m, mimic_o):
        vel = get_diff(position)
        return {'child_parent': self.joint_obj.parent,
                'child_parent_tf': connection_tf * translation3(*(axis[:,:3] * position)) * child_parent_tf,
                'child_pose': parent_pose * connection_tf * translation3(*(axis[:,:3] * position)) * child_pose,
                'connection': self.joint_obj}, \
               {'{}_position'.format(self.conn_path): Constraint(lower_limit - position, upper_limit - position, position),
                '{}_velocity'.format(self.conn_path): Constraint(-vel_limit, vel_limit, vel)}

class SetRevoluteJoint(Operation):
    def init(self, parent_pose, child_pose, connection_path, connection_tf, axis, position, lower_limit, upper_limit, vel_limit, mimic_m=None, mimic_o=None):
        self.joint_obj = RevoluteJoint(str(parent_pose[:-1]), str(child_pose[:-1]), position)
        self.conn_path = connection_path
        op_construction_wrapper(super(SetRevoluteJoint, self).init,
                                'Revolute Joint', 
                                ['child_pose', 'child_parent', 'child_parent_tf'], 
                                (connection_path, 'connection', self.joint_obj),
                                parent_pose=parent_pose,
                                child_parent=child_pose[:-1] + ('parent',),
                                child_parent_tf=child_pose[:-1] + ('to_parent',), 
                                child_pose=child_pose, 
                                connection_tf=connection_tf,
                                axis=axis,
                                position=position,
                                lower_limit=lower_limit,
                                upper_limit=upper_limit,
                                vel_limit=vel_limit,
                                mimic_m=mimic_m,
                                mimic_o=mimic_o)

    def _apply(self, ks, parent_pose, child_pose, child_parent_tf, connection_tf, axis, position, lower_limit, upper_limit, vel_limit, mimic_m, mimic_o):
        position = position if mimic_m is None or mimic_o is None else position * mimic_m + mimic_o
        vel = get_diff(position)
        return {'child_parent': self.joint_obj.parent,
                'child_parent_tf': connection_tf * rotation3_axis_angle(axis, position) * child_parent_tf,
                'child_pose': parent_pose * connection_tf * rotation3_axis_angle(axis, position) * child_pose,
                'connection': self.joint_obj}, \
               {'{}_position'.format(self.conn_path): Constraint(lower_limit - position, upper_limit - position, position),
               '{}_velocity'.format(self.conn_path): Constraint(-vel_limit, vel_limit, vel)}

class SetContinuousJoint(Operation):
    def init(self, parent_pose, child_pose, connection_path, connection_tf, axis, position,vel_limit, mimic_m=None, mimic_o=None):
        self.joint_obj = ContinuousJoint(str(parent_pose[:-1]), str(child_pose[:-1]), position)
        self.conn_path = connection_path
        op_construction_wrapper(super(SetContinuousJoint, self).init,
                                'Continuous Joint', 
                                ['child_pose', 'child_parent', 'child_parent_tf'],
                                (connection_path, 'connection', self.joint_obj),
                                parent_pose=parent_pose, 
                                child_parent=child_pose[:-1] + ('parent',),
                                child_parent_tf=child_pose[:-1] + ('to_parent',),
                                child_pose=child_pose, 
                                connection_tf=connection_tf,
                                axis=axis,
                                position=position,
                                vel_limit=vel_limit,
                                mimic_m=mimic_m,
                                mimic_o=mimic_o) 

    def _apply(self, ks, parent_pose, child_pose, child_parent_tf, connection_tf, axis, position, vel_limit, mimic_m, mimic_o):
        position = position if mimic_m is None or mimic_o is None else position * mimic_m + mimic_o
        return {'child_parent': self.joint_obj.parent,
                'child_parent_tf': connection_tf * rotation3_axis_angle(axis, position) * child_parent_tf,
                'child_pose': parent_pose * connection_tf * rotation3_axis_angle(axis, position) * child_pose,
                'connection': self.joint_obj}, \
               {'{}_velocity'.format(self.conn_path): Constraint(-vel_limit, vel_limit, get_diff(position))}


urdf_geom_types = {urdf.Mesh:     'mesh',
                   urdf.Box:      'box',
                   urdf.Cylinder: 'cylinder',
                   urdf.Sphere:   'sphere'}


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
        

def load_urdf(ks, prefix, urdf, reference_frame='map'):
    if type(prefix) == str:
        prefix = Path(prefix)
        
    ks.apply_operation('create {}'.format(str(prefix)), CreateComplexObject(prefix, URDFRobot(urdf.name)))

    for u_link in urdf.links:
        link_path = prefix + Path(['links', u_link.name])
        collision = None
        geometry  = None
        inertial  = InertialData(link_path, spw.eye(4))
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
            uin = u_link.inertial.inertia # Just for readability
            inertial.inertia_matrix = matrix_wrapper([[uin.ixx, uin.ixy, uin.ixz], 
                                                      [uin.ixy, uin.iyy, uin.iyz],
                                                      [uin.ixz, uin.iyz, uin.izz]])

        ks.apply_operation('create {}'.format(str(prefix + Path(u_link.name))),
                            CreateComplexObject(link_path, 
                                                KinematicLink(reference_frame, urdf_origin_to_transform(u_link.origin), geometry, collision, inertial)))

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
                position   = create_pos((prefix + (u_joint.name, )).to_symbol())
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
            ks.apply_operation('connect {} {}'.format(str(prefix + ('links', u_joint.parent)), str(prefix + ('links', u_joint.child))), op) 

            joint_set.add(n_joint)
            if n_link in urdf.child_map:
                links_left.append(n_link)