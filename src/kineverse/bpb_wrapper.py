import betterpybullet as pb

import kineverse.gradients.common_math as cm

from kineverse.utils import res_pkg_path, real_quat_from_matrix
from kineverse.gradients.gradient_math import GM

# Currently objects created through Python might get deleted accidentally,
# as bullet mostly uses raw pointers, thus not keeping references alive.
# This is a hacky way of maintaining the references.
_collision_shapes = set()

def track_shape(f):
    def wrapper(*args, **kwargs):
        out = f(*args, **kwargs)
        # _collision_shapes.add(out)
        return out
    return wrapper

def matrix_to_transform(matrix):
    quat = real_quat_from_matrix(matrix)
    pos  = matrix[:3,3]
    return pb.Transform(pb.Quaternion(quat[0], quat[1], quat[2], quat[3]), 
                        pb.Vector3(pos[0], pos[1], pos[2]))

def transform_to_matrix(transform, matrix_func=cm.Matrix):
    basis  = transform.basis
    origin = transform.origin
    col_0  = basis.get_col(0)
    col_1  = basis.get_col(1)
    col_2  = basis.get_col(2)
    return matrix_func([[col_0.x, col_1.x, col_2.x, origin.x],
                        [col_0.y, col_1.y, col_2.y, origin.y],
                        [col_0.z, col_1.z, col_2.z, origin.z],
                        [      0,       0,       0,        1]])

@track_shape
def create_cube_shape(extents):
    return pb.BoxShape(pb.Vector3(*[extents[x] * 0.5 for x in range(3)])) if type(extents) is not pb.Vector3 else pb.BoxShape(extents)

@track_shape
def create_cylinder_shape(diameter, height):
    return pb.CylinderShapeZ(pb.Vector3(0.5 * diameter, 0.5 * diameter, height * 0.5))

@track_shape
def create_sphere_shape(diameter):
    return pb.SphereShape(0.5 * diameter)

@track_shape
def create_compound_shape(shapes_poses=[]):
    out = pb.CompoundShape()
    for t, s in shapes_poses:
        out.add_child(t, s)
    return out

# Technically the tracker is not required here, 
# since the loader keeps references to the loaded shapes.
@track_shape
def load_convex_mesh_shape(pkg_filename):
    return pb.load_convex_shape(res_pkg_path(pkg_filename))


def create_object(shape, transform=pb.Transform.identity()):
    if type(transform) is not pb.Transform:
        if type(transform) is GM:
            transform = transform.to_sym_matrix()
        transform = matrix_to_transform(transform)
    out = pb.CollisionObject()
    out.collision_shape = shape
    out.collision_flags = pb.CollisionObject.KinematicObject
    out.transform = transform
    return out

def create_cube(extents, transform=pb.Transform.identity()):
    return create_object(create_cube_shape(extents), transform)

def create_sphere(diameter, transform=pb.Transform.identity()):
    return create_object(create_sphere_shape(diameter), transform)

def create_cylinder(diameter, height, transform=pb.Transform.identity()):
    return create_object(create_cylinder_shape(diameter, height), transform)

def create_compund_object(shapes_transforms, transform=pb.Transform.identity()):
    return create_object(create_compound_shape(shapes_transforms), transform)

def create_convex_mesh(pkg_filename, transform=pb.Transform.identity()):
    return create_object(load_convex_mesh_shape(pkg_filename), transform)
