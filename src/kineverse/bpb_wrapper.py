import betterpybullet as pb

from kineverse.utils import res_pkg_path, real_quat_from_matrix
from kineverse.gradients.gradient_math import GM

# Currently objects created through Python might get deleted accidentally,
# as bullet mostly uses raw pointers, thus not keeping references alive.
# This is a hacky way of maintaining the references.
_collision_shapes = set()

def track_shape(f):
    def wrapper(*args, **kwargs):
        out = f(*args, **kwargs)
        _collision_shapes.add(out)
        return out
    return wrapper


@track_shape
def create_cube_shape(extents):
    return pb.BoxShape(pb.Vector3(*[x * 0.5 for x in extents[:3]])) if type(extents) is not pb.Vector3 else pb.BoxShape(extents)

@track_shape
def create_cylinder_shape(diameter, height):
    return pb.CylinderShapeZ(pb.Vector3(0.5 * diameter, 0.5 * diameter, height * 0.5))

@track_shape
def create_sphere_shape(diameter):
    return pb.SphereShape(0.5 * diameter)

# Technically the tracker is not required here, 
# since the loader keeps references to the loaded shapes.
@track_shape
def load_convex_mesh_shape(pkg_filename):
    return pb.load_convex_shape(res_pkg_path(pkg_filename))


def create_object(shape, transform=pb.Transform.identity()):
    if type(transform) is not pb.Transform:
        if type(transform) is GM:
            transform = transform.to_sym_matrix()
        quat = real_quat_from_matrix(transform)
        pos  = pos_of(transform)[:3]
        transform = pb.Transform(pb.Quaternion(*quat), pb.Vector3(*pos))
    out = pb.CollisionObject()
    out.set_collision_shape(shape)
    out.collision_flags = pb.CollisionObject.KinematicObject
    out.transform = transform
    return out

def create_cube(extents, transform=pb.Transform.identity()):
    return create_object(create_cube_shape(extents), transform)

def create_sphere(diameter, transform=pb.Transform.identity()):
    return create_object(create_sphere_shape(diameter), transform)

def create_cylinder(diameter, height, transform=pb.Transform.identity()):
    return create_object(create_cylinder_shape(diameter, height), transform)

def create_convex_mesh(pkg_filename, transform=pb.Transform.identity()):
    return create_object(load_convex_mesh_shape(pkg_filename), transform)