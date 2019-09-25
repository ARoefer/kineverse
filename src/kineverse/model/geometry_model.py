import re
import numpy as np

from giskardpy import BACKEND
from kineverse.gradients.diff_logic    import Position
from kineverse.gradients.gradient_math import *
from kineverse.json_wrapper            import JSONSerializable
from kineverse.model.paths             import Path, PathSet, PathDict
from kineverse.model.kinematic_model   import Constraint
from kineverse.model.event_model       import EventModel
from kineverse.model.frames            import Frame
from kineverse.bpb_wrapper             import pb, create_object, create_cube_shape, create_sphere_shape, create_cylinder_shape, create_compound_shape, load_convex_mesh_shape, matrix_to_transform


class KinematicJoint(JSONSerializable):
    def __init__(self, jtype, parent, child):
        self.type     = jtype
        self.parent   = parent
        self.child    = child

    def _json_data(self, json_dict):
        json_dict.update({'jtype':  self.type,
                          'parent': self.parent,
                          'child':  self.child})

    def __deepcopy__(self, memo):
        out = KinematicLink(self.type, self.parent, self.child)
        memo[id(self)] = out
        return out

class Geometry(Frame):
    def __init__(self, parent_path, pose, geom_type, scale=None, mesh=None):
        super(Geometry, self).__init__(parent_path, pose)
        self.type  = geom_type
        self.scale = scale if scale is not None else vector3(1,1,1)
        self.mesh  = mesh

    def _json_data(self, json_dict):
        super(Geometry, self)._json_data(json_dict)
        del json_dict['to_parent']
        json_dict.update({'geom_type': self.type,
                          'scale':     self.scale,
                          'mesh':      self.mesh})

class InertialData(Frame):
    def __init__(self, parent_path, pose, mass=1, inertia_matrix=spw.eye(3)):
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


class RigidBody(Frame):
    def __init__(self, parent_path, pose, geometry=None, collision=None, inertial=None):
        super(RigidBody, self).__init__(parent_path, pose)
        self.geometry  = geometry
        self.collision = collision
        self.inertial  = inertial

    def _json_data(self, json_dict):
        super(RigidBody, self)._json_data(json_dict)
        del json_dict['to_parent']
        json_dict.update({'collision': self.collision,
                          'geometry':  self.geometry,
                          'inertial':  self.inertial})

    def __deepcopy__(self, memo):
        out = RigidBody(self.parent, self.pose * 1, self.geometry, self.collision, self.inertial)
        memo[id(self)] = out
        return out


class ArticulatedObject(JSONSerializable):
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
        out = ArticulatedObject(self.name)
        memo[id(self)] = out
        out.links  = {k: deepcopy(v) for k, v in self.links.items()}
        out.joints = {k: deepcopy(v) for k, v in self.joints.items()}
        return out


obj_to_obj_prefix = 'distance_obj_to_obj'
obj_to_obj_infix  = 'UND'
obj_to_world_prefix = 'distance_obj_to_world'

def create_distance_symbol(obj_path, other_path=None):
    return '{}{}{}{}'.format(obj_to_obj_prefix, str(obj_path), obj_to_obj_infix, str(other_path)) if other_path is not None else '{}{}'.format(obj_to_world_prefix, str(obj_path))


class GeometryModel(EventModel):
    def __init__(self):
        super(GeometryModel, self).__init__()

        self.kw = pb.KineverseWorld()
        self.kw.force_update_all_aabbs = True
        self._bodies_process = PathDict()
        self._collision_objects = {}
        self._co_pose_expr = {}
        self._co_symbol_map = {}
        self._symbol_co_map = {}
        self._co_callbacks  = {}

    def _process_link_insertion(self, key, link):
        if link.collision is not None and str(key) not in self._collision_objects:
            shape = create_compound_shape()
            for c in link.collision.values():
                if c.type == 'mesh':
                    sub_shape = load_convex_mesh_shape(c.mesh)
                    shape.add_child(matrix_to_transform(c.to_parent), sub_shape)
                elif c.type == 'box':
                    sub_shape = create_cube_shape(c.scale)
                    shape.add_child(matrix_to_transform(c.to_parent), sub_shape)
                elif c.type == 'cylinder':
                    sub_shape = create_cylinder_shape(c.scale[0], c.scale[2])
                    shape.add_child(matrix_to_transform(c.to_parent), sub_shape)
                elif c.type == 'sphere':
                    sub_shape = create_sphere_shape(c.scale[0])
                    shape.add_child(matrix_to_transform(c.to_parent), sub_shape)
                else:
                    raise Exception('Unrecognized geometry type in collision of link {}. Type is "{}"'.format(str(key), c.type))
            body = create_object(shape)
            self.kw.add_collision_object(body)
            self._collision_objects[str(key)] = body
            self._co_symbol_map[str(key)] = set()
            print('Created collision representation for {}'.format(key))

            #self.register_on_model_changed(key, update_collision_object)


    def _process_joint_insertion(self, key, joint):
        if joint.parent in self._collision_objects and joint.child in self._collision_objects:
            parent = self._collision_objects[joint.parent]
            child  = self._collision_objects[joint.parent]
            parent.set_ignore_collision(child)
            child.set_ignore_collision(parent)

            def update_joint_object(model):
                if model is None:
                    self._process_joint_removal(joint)

    def _process_link_removal(self, key):
        body = self._collision_objects[str(key)]
        self.kw.remove_collision_object(body)
        for s in self._co_symbol_map[str(key)]:
            self._symbol_co_map[s].discard(body)
        del self._co_symbol_map[str(key)]
        del self._collision_objects[str(key)]

    def _process_joint_removal(self, joint):
        if joint.parent in self._collision_objects and joint.child in self._collision_objects:
            parent = self._collision_objects[joint.parent]
            child  = self._collision_objects[joint.parent]
            parent.set_ignore_collision(child)
            child.set_ignore_collision(parent)

    def set_data(self, key, value):
        if type(key) is str:
            key = Path(key)

        if isinstance(value, RigidBody):
            self._process_link_insertion(key, value)
        elif isinstance(value, KinematicJoint):
            self._process_joint_insertion(key, value)
        elif isinstance(value, ArticulatedObject):
            for lname, link in value.links.items():
                self._process_link_insertion(key + ('links', lname,), link)
            for jname, joint in value.joints.items():
                self._process_joint_insertion(key + ('joints', jname,), joint)
        super(GeometryModel, self).set_data(key, value)

    def remove_data(self, key):
        # if str(key) in self._collision_objects:
        #     self._process_link_removal(key)
        # elif self.has_data(key):
        #     data = self.get_data(key)
        #     if isinstance(data, KinematicJoint):
        #         self._process_joint_removal(data)
        #     elif isinstance(data, URDFRobot):
        #         for joint in value.joints.values():
        #             self._process_joint_removal(joint)
        #         for lname in value.links.keys():
        #             self._process_link_removal(key + (lname,))
        super(GeometryModel, self).remove_data(key)

    def dispatch_events(self):
        static_objects = []
        static_poses   = []
        for str_k in self._collision_objects:
            k = Path(str_k)
            if k in self._callback_batch:
                #print('{} is a collision link and has changed in the last update batch'.format(k))
                if self.has_data(k):
                    #print('{} was changed'.format(k))
                    link = self.get_data(k)
                    pose_expr = link.pose # * link.collision.to_parent
                    if type(pose_expr) is GM:
                        pose_expr = pose_expr.to_sym_matrix()
                    self._co_pose_expr[str_k]  = pose_expr
                    self._co_symbol_map[str_k] = pose_expr.free_symbols
                    if len(pose_expr.free_symbols) > 0:
                        for s in pose_expr.free_symbols:
                            if s not in self._symbol_co_map:
                                self._symbol_co_map[s] = set()
                            self._symbol_co_map[s].add(str_k)
                    else:
                        #print('{} is static, setting its pose to:\n{}'.format(k, pose_expr))
                        static_objects.append(self._collision_objects[str_k])
                        static_poses.append(np.array(pose_expr.tolist()))
                else:
                    self._process_link_removal(k)
        if len(static_objects) > 0:
            self.kw.batch_set_transforms(static_objects, np.vstack(static_poses))
            # print('\n  '.join(['{}: {}'.format(n, c.transform) for n, c in self._collision_objects.items()]))
        super(GeometryModel, self).dispatch_events()


    def get_constraints_by_symbols(self, symbol_set, n_dist_constraints=1):
        out = {}
        # for s in symbol_set:
        #     if str(s)[:len(obj_to_obj_prefix)] == obj_to_obj_prefix:
        #         obj_a, obj_b = str(s)[len(obj_to_obj_prefix):].split(obj_to_obj_infix)
        #         obj_a = str(Path(obj_a))
        #         obj_b = str(Path(obj_b))
        #         if obj_a not in self._co_pose_expr:
        #             raise Exception('Object A "{}" is not a physical object that a distance constraint can be generated for.'.format(obj_a))
        #         if obj_b not in self._co_pose_expr:
        #             raise Exception('Object B "{}" is not a physical object that a distance constraint can be generated for.'.format(obj_a))
        #         out['minimal distance {} {}'.format(obj_a, obj_b)] = ClosestDistanceConstraint(self._co_pose_expr[obj_a], self._co_pose_expr[obj_b], Path(obj_a), Path(obj_b))

        #     elif str(s)[:len(obj_to_world_prefix)] == obj_to_world_prefix:
        #         obj_a = str(Path(str(s)[len(obj_to_world_prefix):]))
        #         if obj_a not in self._co_pose_expr:
        #             raise Exception('Can not generate minimal distance constraint for object "{}"'.format(obj_a))
        #         obj_pose = self._co_pose_expr[obj_a]
        #         handler = ContactHandler(Path(obj_a))
        #         for x in range(n_dist_constraints):
        #             handler.add_passive_handle()
        #             c = handler.var_map[x]
        #             out['minimal distance {} WORLD {}'.format(obj_a, x)] = ClosestDistanceConstraint(obj_pose, spw.eye(4), Path(obj_a), Path('world/{}'.format(x)))
        out.update(super(GeometryModel, self).get_constraints_by_symbols(symbol_set))
        return out


    def get_active_geometry(self, symbols, static_state=None):
        coll_obj_keys = set()
        for s in symbols:
            if s in self._symbol_co_map:
                coll_obj_keys.update(self._symbol_co_map[s])

        obj_names = sorted(coll_obj_keys)
        if len(obj_names) == 0:
            return CollisionSubworld(self.kw, [], [], set(), None)

        objs = [self._collision_objects[n] for n in obj_names]
        pose_matrix = self._co_pose_expr[obj_names[0]]
        for n in obj_names[1:]:
            pose_matrix = pose_matrix.col_join(self._co_pose_expr[n])

        if static_state is not None:
            pose_matrix = pose_matrix.subs({s: static_state[s] for s in pose_matrix.free_symbols.difference(symbols) if s in static_state})

        cythonized_matrix = spw.speed_up(pose_matrix, pose_matrix.free_symbols, backend=BACKEND)

        return CollisionSubworld(self.kw, obj_names, objs, pose_matrix.free_symbols, cythonized_matrix)


class CollisionSubworld(object):
    def __init__(self, world, names, collision_objects, free_symbols, pose_generator):
        self.world = world
        self.names = names
        self.collision_objects = collision_objects
        self.named_objects     = dict(zip(self.names, self.collision_objects))
        self.object_name_map   = {o: n for n, o in self.named_objects.items()}
        self.free_symbols      = free_symbols
        self.pose_generator    = pose_generator
        self._state            = {}
        self._needs_update     = False 
        self._contacts         = None

    @profile
    def update_world(self, state):
        self._state.update({str(s): v for s, v in state.items() if s in self.free_symbols})
        #print('Subworld state: \n {}'.format('\n '.join(['{:>20}: {}'.format(s, v) for s, v in self._state.items()])))
        if self.pose_generator != None:
            self.world.batch_set_transforms(self.collision_objects, self.pose_generator(**self._state))
            self._needs_update = True

    @property
    def contacts(self):
        if self._needs_update:
            self.world.perform_discrete_collision_detection()
            self._contacts = self.world.get_contacts()
        return self._contacts

    @profile
    def closest_distances(self, query_batch):
        if self._needs_update:
            self.world.update_aabbs()
        return self.world.get_closest_batch(query_batch)



def closest_distance(pose_a, pose_b, path_a, path_b):
    cont = ContactSymbolContainer(path_a, path_b)
    point_a = pose_a * point3(cont.on_a_x, cont.on_a_y, cont.on_a_z)
    point_b = pose_b * point3(cont.on_b_x, cont.on_b_y, cont.on_b_z)
    return norm(point_a - point_b)

def closest_distance_constraint(obj_a_pose, obj_b_pose, obj_a_path, obj_b_path, margin=0):
    dist = closest_distance(obj_a_pose, obj_b_pose, obj_a_path, obj_b_path)
    return Constraint(margin - dist, 1000, dist)

def contact_constraint(obj_a_pose, obj_b_pose, obj_a_path, obj_b_path):
    dist = closest_distance(obj_a_pose, obj_b_pose, obj_a_path, obj_b_path)
    return Constraint(-dist, -dist, dist)


class ContactSymbolContainer(object):
    def __init__(self, path_obj, path_other=0):
        contact_name = ('contact',) + path_obj + (obj_to_obj_infix,) + path_other
        self.on_a_x = Position((contact_name + ('onA', 'x')).to_symbol())
        self.on_a_y = Position((contact_name + ('onA', 'y')).to_symbol())
        self.on_a_z = Position((contact_name + ('onA', 'z')).to_symbol())
        self.on_b_x = Position((contact_name + ('onB', 'x')).to_symbol())
        self.on_b_y = Position((contact_name + ('onB', 'y')).to_symbol())
        self.on_b_z = Position((contact_name + ('onB', 'z')).to_symbol())

pb_zero_vector = pb.Vector3(0,0,0)
pb_far_away_vector = pb.Vector3(1e8,1e8,-1e8)

class ContactHandler(object):
    def __init__(self, object_path):
        self.obj_path = object_path
        self.state    = {}
        self.var_map  = {}
        self._num_anon_contacts = 0

    @property
    def num_anon_contacts(self):
        return self._num_anon_contacts
    

    def has_handle_for(self, other_path):
        return other_path in self.var_map

    def add_active_handle(self, other_path):
        if other_path not in self.var_map:
            self.var_map[other_path] = ContactSymbolContainer(self.obj_path, other_path)
            self.handle_contact(pb_zero_vector, pb_zero_vector, other_path)

    def add_passive_handle(self):
        self.var_map[self._num_anon_contacts] = ContactSymbolContainer(self.obj_path, Path('anon/{}'.format(self._num_anon_contacts)))
        self.handle_contact(pb_zero_vector, pb_zero_vector, self._num_anon_contacts)
        self._num_anon_contacts += 1


    @profile
    def handle_contact(self, point_a, point_b, name=0):
        if name not in self.var_map:
            raise Exception('Name {} is not known to contact handler'.format(name))
        container = self.var_map[name]
        self.state.update({container.on_a_x: point_a.x,
                           container.on_a_y: point_a.y,
                           container.on_a_z: point_a.z,
                           container.on_b_x: point_b.x,
                           container.on_b_y: point_b.y,
                           container.on_b_z: point_b.z})

    @profile
    def handle_contacts(self, contacts, name_resolver=None):
        anon_idx = 0
        for cp in contacts:
            if len(cp.points) > 0:
                if name_resolver is not None and cp.obj_b in name_resolver and name_resolver[cp.obj_b] in self.var_map:
                    name = name_resolver[cp.obj_a]
                    self.handle_contact(cp.points[0].point_a, cp.points[0].point_b, name)
                elif anon_idx < self._num_anon_contacts:
                    self.handle_contact(cp.points[0].point_a, cp.obj_b.transform * cp.points[0].point_b, anon_idx)
                    anon_idx += 1
        
        if anon_idx < self._num_anon_contacts:
            for x in range(anon_idx, self._num_anon_contacts):
                self.handle_contact(pb_zero_vector, pb_far_away_vector, anon_idx)







