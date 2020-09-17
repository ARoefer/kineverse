import betterpybullet as pb
import math


from kineverse.visualization.ros_visualizer import ROSVisualizer
from kineverse.utils import make_pkg_path


cylinder_rotations = [pb.Quaternion(0, math.pi * 0.5, 0), pb.Quaternion(0, 0, math.pi * 0.5), pb.Quaternion.identity()]

class ROSBPBVisualizer(ROSVisualizer):
    def __init__(self, vis_topic, base_frame='base_link'):
        super(ROSBPBVisualizer, self).__init__(vis_topic, base_frame)

        self.layer_drawn_objects = {}
        self._cached_file_paths  = {}

    def begin_draw_cycle(self, *layers):
        super(ROSBPBVisualizer, self).begin_draw_cycle(*layers)

        if len(layers) > 0:
            self.layer_drawn_objects.update({l: set() for l in layers})
        else:
            self.layer_drawn_objects = {l: set() for l in self.layer_drawn_objects.keys()}

    def draw_collision_shape(self, namespace, shape, transform, r=1, g=1, b=1, a=1, frame=None):
        if isinstance(shape, pb.BoxShape):
            self.draw_cube(namespace, transform, shape.extents, r, g, b, a, frame)
        elif isinstance(shape, pb.SphereShape):
            self.draw_sphere(namespace, transform.origin, shape.radius, r,g, b, a, frame)
        elif isinstance(shape, pb.CylinderShape):
            transform = transform * pb.Transform(cylinder_rotations[shape.axis]) 
            self.draw_cylinder(namespace, transform, shape.height, shape.radius, r, g, b, a, frame)
        elif isinstance(shape, pb.CompoundShape):
            if shape.file_path != '':
                if shape.file_path not in self._cached_file_paths:
                    self._cached_file_paths[shape.file_path] = make_pkg_path(shape.file_path)
                self.draw_mesh(namespace, transform, shape.scaling, self._cached_file_paths[shape.file_path], frame, r, g, b, a)
            else:
                for x in range(shape.nchildren):
                    self.draw_collision_shape(namespace, shape.get_child(x), transform * shape.get_child_transform(x), r, g, b, a, frame)
        elif isinstance(shape, pb.ConvexHullShape):
            if shape.file_path != '':
                if shape.file_path not in self._cached_file_paths:
                    self._cached_file_paths[shape.file_path] = make_pkg_path(shape.file_path)
                self.draw_mesh(namespace, transform, shape.scaling, self._cached_file_paths[shape.file_path], frame, r, g, b, a)


    def draw_collision_object(self, namespace, obj, r=1, g=1, b=1, a=1, frame=None):
        #if namespace not in self.layer_drawn_objects or obj not in self.layer_drawn_objects[namespace]:

        self.draw_collision_shape(namespace, obj.collision_shape, obj.transform, r, g, b, a, frame)

        if not namespace in self.layer_drawn_objects:
            self.layer_drawn_objects[namespace] = set()
        self.layer_drawn_objects[namespace].add(obj)


    def draw_world(self, namespace, world, r=1, g=1, b=1, a=1, frame=None):
        for obj in world.collision_objects:
            self.draw_collision_object(namespace, obj, r, g, b, a, frame=frame)


    # def draw_subworld(self, namespace, sub_world, frame=None):
    #     for obj in sub_world.collision_objects:
    #         self.draw_collision_object(namespace, obj, frame)


    def draw_contacts(self, namespace, contacts, size, r=1, g=0, b=0, a=1, arrow_length=1.0, frame=None):
        for cp in contacts:
            lines = sum([[cp.obj_a.np_transform.dot(p.point_a), cp.obj_b.np_transform.dot(p.point_b)] for p in cp.points], [])
            self.draw_lines(namespace, pb.Transform.identity(), size, lines, r, g, b, a, frame)
            self.draw_points(namespace, pb.Transform.identity(), size * 2, lines, r, g, b, a, frame)
            for x, p in enumerate(cp.points):
                self.draw_vector(namespace, lines[x * 2 + 1], p.normal_world_b * arrow_length, r, g, b, a, frame=frame)
