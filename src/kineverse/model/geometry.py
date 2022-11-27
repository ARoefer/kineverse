import kineverse.gradients.gradient_math as gm

from kineverse.json_serializable import JSONSerializable
from kineverse.model.frames      import Frame
from kineverse.utils             import rot3_to_rpy


class KinematicJoint(JSONSerializable):
    """A representation of a joint. Used to identify the connectedness
    of two objects.
    """
    def __init__(self, jtype, parent, child):
        self.type     = jtype
        self.parent   = parent
        self.child    = child

    def _json_data(self, json_dict):
        json_dict.update({'jtype':  self.type,
                          'parent': self.parent,
                          'child':  self.child})

    def __deepcopy__(self, memo):
        out = type(self)(self.type, self.parent, self.child)
        memo[id(self)] = out
        return out

    def __eq__(self, other):
        if isinstance(other, KinematicJoint):
            return self.type == other.type and self.parent == other.parent and self.child == other.child
        return False

class Geometry(Frame):
    """Representation of all geometry."""
    def __init__(self, parent_path, pose, geom_type, scale=None, mesh=None):
        super(Geometry, self).__init__(parent_path, pose)
        self.type  = geom_type
        self.scale = scale if scale is not None else gm.vector3(1,1,1)
        self.mesh  = mesh

    def _json_data(self, json_dict):
        super(Geometry, self)._json_data(json_dict)
        del json_dict['to_parent']
        json_dict.update({'geom_type': self.type,
                          'scale':     self.scale,
                          'mesh':      self.mesh})

    def to_parent_xyz_str(self):
        pos = gm.pos_of(self.to_parent)
        return ' '.join([str(pos[x]) for x in range(3)])
    
    def to_parent_rpy_str(self):
        rot = rot3_to_rpy(self.to_parent, True)
        return ' '.join([str(rot[x]) for x in range(3)])

    def __eq__(self, other):
        if isinstance(other, Geometry):
            return super(Geometry, self).__eq__(other) and \
                   self.type == other.type and \
                   gm.eq_expr(self.scale, other.scale) and \
                   self.mesh == other.mesh
        return False

GEOM_TYPE_MESH     = 'mesh'
GEOM_TYPE_BOX      = 'box'
GEOM_TYPE_CYLINDER = 'cylinder'
GEOM_TYPE_SPHERE   = 'sphere'

def Box(parent_path, pose, scale=None):
    return Geometry(parent_path, pose, GEOM_TYPE_BOX, scale, None)

def Mesh(parent_path, pose, mesh, scale=None):
    return Geometry(parent_path, pose, GEOM_TYPE_MESH, scale, mesh)

def Cylinder(parent_path, pose, diameter, height):
    return Geometry(parent_path, pose, GEOM_TYPE_CYLINDER, gm.vector3(diameter, diameter, height), None)

def Sphere(parent_path, pose, radius):
    return Geometry(parent_path, pose, GEOM_TYPE_SPHERE, gm.vector3(radius, radius, radius), None)


class InertialData(Frame):
    """Unused."""
    def __init__(self, parent_path, pose, mass=1, inertia_matrix=gm.eye(3)):
        super(InertialData, self).__init__(parent_path, pose)
        if mass < 0:
            raise Exception('Mass can not be negative!')

        self.mass = mass
        self.inertia_matrix = inertia_matrix

    def _json_data(self, json_dict):
        super(InertialData, self)._json_data(json_dict)
        del json_dict['to_parent']
        json_dict.update({'mass':           self.mass,
                          'inertia_matrix': self.inertia_matrix})

    def __eq__(self, other):
        if isinstance(other, InertialData):
            return super(InertialData, self).__eq__(other) and \
                   self.mass == other.mass and \
                   gm.eq_expr(self.inertia_matrix, other.inertia_matrix)
        return False


class RigidBody(Frame):
    """Representation of a rigid body in the URDF-sense. Consists of visual and collision geometry.
    """
    def __init__(self, parent_path, pose, to_parent=None, geometry=None, collision=None, inertial=None, parent_joint=None):
        super(RigidBody, self).__init__(parent_path, pose, to_parent)
        self.geometry  = geometry
        self.collision = collision
        self.inertial  = inertial
        self.parent_joint = parent_joint

    def _json_data(self, json_dict):
        super(RigidBody, self)._json_data(json_dict)
        json_dict.update({'collision': self.collision,
                          'geometry':  self.geometry,
                          'inertial':  self.inertial,
                          'parent_joint': self.parent_joint})

    def __deepcopy__(self, memo):
        out = type(self)(self.parent, self.pose * 1, self.to_parent * 1, self.geometry, self.collision, self.inertial, self.parent_joint)
        memo[id(self)] = out
        return out

    def __eq__(self, other):
        if isinstance(other, RigidBody):
            return super(RigidBody, self).__eq__(other) and \
                   self.geometry == other.geometry and \
                   self.collision == other.collision and \
                   self.inertial == other.inertial and \
                   self.parent_joint == other.parent_joint
        return False

class ArticulatedObject(JSONSerializable):
    """An articulated object is a collection of rigid bodies and joints."""
    def __init__(self, name):
        self.name   = name
        self.links  = {}
        self.joints = {}

    def _json_data(self, json_dict):
        json_dict.update({'name':   self.name,
                          'links':  self.links, 
                          'joints': self.joints})

    @classmethod
    def json_factory(cls, name, links, joints):
        out = cls(name)
        out.links  = links
        out.joints = joints
        return out

    def __deepcopy__(self, memo):
        out = type(self)(self.name)
        memo[id(self)] = out
        out.links  = {k: copy.deepcopy(v) for k, v in self.links.items()}
        out.joints = {k: copy.deepcopy(v) for k, v in self.joints.items()}
        return out

    def __eq__(self, other):
        if isinstance(other, ArticulatedObject):
            return self.name == other.name and self.links == other.links and self.joints == other.joints
        return False

