"""
The geometry_model module provides a third articulation model that is derived from the EventModel.
This model recognizes the insertion of rigid bodies and builds up a collision scene in the background.
"""
import numpy as np
import copy

import kineverse.gradients.gradient_math as gm

from kineverse.gradients.diff_logic     import Symbol, Position
from kineverse.json_wrapper             import JSONSerializable
from kineverse.bpb_wrapper              import pb,                     \
                                               create_object,          \
                                               create_cube_shape,      \
                                               create_sphere_shape,    \
                                               create_cylinder_shape,  \
                                               create_compound_shape,  \
                                               load_convex_mesh_shape, \
                                               matrix_to_transform
from kineverse.utils                    import rot3_to_rpy

from .paths                             import Path, PathSet, PathDict
from .articulation_model                import Constraint
from .event_model                       import EventModel
from .geometry                          import Frame, \
                                               Geometry, \
                                               Box, \
                                               Mesh, \
                                               Cylinder, \
                                               Sphere, \
                                               KinematicJoint, \
                                               RigidBody, \
                                               ArticulatedObject
from . import model_settings


obj_to_obj_prefix = 'distance_obj_to_obj'
obj_to_obj_infix  = 'UND'
obj_to_world_prefix = 'distance_obj_to_world'

def create_distance_symbol(obj_path, other_path=None):
    """Helper to create symbols that identify geometry queries."""
    return f'{obj_to_obj_prefix}' \
           f'{str(obj_path)}'     \
           f'{obj_to_obj_infix}'  \
           f'{str(other_path)}' if other_path is not None else f'{obj_to_world_prefix}{str(obj_path)}'


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
        self._static_objects = []

    def _process_body_insertion(self, key, link):
        """Creates a rigid body for the given link, if it has collision geometry."""
        if link.collision is not None and key not in self._collision_objects:
            shape = create_compound_shape()
            for c in link.collision.values():
                if c.type == 'mesh':
                    sub_shape = load_convex_mesh_shape(c.mesh, False, c.scale)
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
                    raise Exception('Unrecognized geometry type in collision of link {}. Type is "{}"'.format(key, c.type))
            body = create_object(shape)
            self.kw.add_collision_object(body)
            self._collision_objects[key] = body
            self._co_symbol_map[key] = set()
            # print('Created collision representation for {}'.format(key))

            #self.register_on_model_changed(key, update_collision_object)


    def _process_joint_insertion(self, key, joint):
        """Sets the joint's child and parent object to ignore each other in collision."""
        if joint.parent in self._collision_objects and joint.child in self._collision_objects:
            parent = self._collision_objects[joint.parent]
            child  = self._collision_objects[joint.parent]
            parent.set_ignore_collision(child)
            child.set_ignore_collision(parent)

            def update_joint_object(model):
                if model is None:
                    self._process_joint_removal(joint)

    def _process_link_removal(self, key):
        """Removes a collision object from the collision world."""
        body = self._collision_objects[key]
        self.kw.remove_collision_object(body)
        for s in self._co_symbol_map[key]:
            self._symbol_co_map[s].discard(key)
        del self._co_symbol_map[key]
        del self._collision_objects[key]

    def _process_joint_removal(self, joint):
        """Removes the collision ignore-flag of a joint's child and parent."""
        if joint.parent in self._collision_objects and joint.child in self._collision_objects:
            parent = self._collision_objects[joint.parent]
            child  = self._collision_objects[joint.parent]
            parent.set_ignore_collision(child)
            child.set_ignore_collision(parent)

    def set_data(self, key, value):
        if type(key) is str:
            key = Path(key)

        if isinstance(value, RigidBody):
            self._process_body_insertion(key, value)
        elif isinstance(value, KinematicJoint):
            self._process_joint_insertion(key, value)
        elif isinstance(value, ArticulatedObject):
            for lname, link in value.links.items():
                self._process_body_insertion(key + ('links', lname,), link)
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
        """Extension of dispatch_events. Collects the FK expressions of all collision objects."""
        static_objects = []
        static_poses   = []
        dynamic_poses  = {}
        removed_object = []
        for k_path in self._collision_objects: # type: Path
            if model_settings.BRUTE_MODE or k_path in self._callback_batch:
                #print('{} is a collision link and has changed in the last update batch'.format(k))
                if self.has_data(k_path):
                    #print('{} was changed'.format(k))
                    link = self.get_data(k_path)
                    pose_expr  = link.pose # * link.collision.to_parent
                    symbol_set = set()
                    if type(pose_expr) is gm.GM:
                        symbol_set  = symbol_set.union(pose_expr.diff_symbols)
                        pose_expr   = pose_expr.to_sym_matrix()
                    # fixme, this is a hack
                    symbol_set |= gm.free_symbols(pose_expr) # type: {Symbol}
                    if k_path in self._co_symbol_map: # The body might have been associated with different symbols before
                        for s_discard in self._co_symbol_map[k_path].difference(symbol_set):
                            self._symbol_co_map[s_discard].remove(k_path)

                    self._co_pose_expr[k_path]  = pose_expr
                    self._co_symbol_map[k_path] = symbol_set
                    if len(symbol_set) > 0:
                        for s in symbol_set:
                            if s not in self._symbol_co_map:
                                self._symbol_co_map[s] = set()
                            self._symbol_co_map[s].add(k_path)
                        dynamic_poses[k_path] = gm.to_numpy(gm.subs(pose_expr, {s: 0 for s in gm.free_symbols(pose_expr)}))
                        # print('Evaluation of {} yields:\n{}'.format(pose_expr, dynamic_poses[k_path]))
                    else:
                        # print('{} is static, setting its pose to:\n{}'.format(k, pose_expr))
                        static_objects.append(self._collision_objects[k_path])
                        static_poses.append(gm.to_numpy(pose_expr))
                else:
                    removed_object.append(k_path)
        for k_path in removed_object:
            self._process_link_removal(k_path)
        if len(static_objects) > 0:
            pb.batch_set_transforms(static_objects, np.vstack(static_poses))
            self._static_objects = static_objects
            # print('\n  '.join(['{}:\n{}'.format(n, c.transform) for n, c in self._collision_objects.items()]))

        if len(dynamic_poses) > 0:
            objs, matrices = zip(*[(self._collision_objects[k], m) for k, m in dynamic_poses.items()])
            pb.batch_set_transforms(objs, np.vstack(matrices))

        super(GeometryModel, self).dispatch_events()


    def get_constraints_by_symbols(self, symbol_set, n_dist_constraints=1):
        out = {}
        out.update(super(GeometryModel, self).get_constraints_by_symbols(symbol_set))
        return out

    def get_active_geometry_raw(self, symbols):
        """Returns the collision objects whose FK-expressions are affected by the given symbol set."""
        return {k: self._collision_objects[k] for k in set(sum([list(self._symbol_co_map[s]) 
                                              for s in symbols 
                                              if s in self._symbol_co_map], []))}

    def get_active_geometry(self, symbols, static_state=None, include_static=True):
        """Creates a so-called CollisionSubworld, which is a wrapped collision world which is focused on
        the objects affected by the given symbol set, and static objects.

        :param symbols: Symbols that affect the FK-expressions of the world.
        :param static_state: Substitution map to apply to the FK-expressions. (Does not work in casadi)
        :param include_static: Should static geometry be included?
        :return: CollisionSubworld
        """
        coll_obj_keys = set()
        for s in symbols:
            if s in self._symbol_co_map:
                coll_obj_keys.update(self._symbol_co_map[s])

        obj_names = sorted(coll_obj_keys)
        if len(obj_names) == 0:
            return CollisionSubworld(self.kw, [], [], set(), None)

        objs = [self._collision_objects[n] for n in obj_names]
        pose_matrix = gm.vstack(*[self._co_pose_expr[n] for n in obj_names])

        if static_state is not None:
            pose_matrix = gm.subs(pose_matrix, {s: static_state[s] for s in gm.free_symbols(pose_matrix).difference(symbols) if s in static_state})

        cythonized_matrix = gm.speed_up(pose_matrix, gm.free_symbols(pose_matrix))

        # world = pb.KineverseWorld()
        # for obj in filtered_objs:
        #     world.add_collision_object(obj)

        if include_static:
            return CollisionSubworld(self.kw, obj_names, objs, 
                                     gm.free_symbols(pose_matrix), cythonized_matrix, self._static_objects.copy())
        return CollisionSubworld(self.kw, obj_names, objs, 
                                 gm.free_symbols(pose_matrix), cythonized_matrix)


class CollisionSubworld(object):
    """A wrapper around the bpb collision world which is focused on a subset of objects."""
    def __init__(self, world, names, dynamic_objects, free_symbols, pose_generator, static_objects=[]):
        self.world = world
        self.names = names
        self.dynamic_objects = dynamic_objects
        self.static_objects  = static_objects
        self.named_objects   = dict(zip(self.names, self.dynamic_objects))
        self.object_name_map = {o: n for n, o in self.named_objects.items()}
        self.free_symbols    = free_symbols
        self.pose_generator  = pose_generator
        self._state          = {}
        self._needs_update   = False 
        self._contacts       = None

    @profile
    def update_world(self, state):
        """Update the objects' poses with a Substitution map."""
        self.update_world_unchecked({str(s): v for s, v in state.items() if s in self.free_symbols})

    @profile
    def update_world_unchecked(self, state):
        """Update the objects' poses with a Substitution map. 
        THIS IS UNCHECKED, USE AT YOU OWN DISGRESSION
        
        :param state: Substitution map, mapping strings to values.
        """
        self._state.update(state)
        if self.pose_generator != None:
            pb.batch_set_transforms(self.dynamic_objects, self.pose_generator(**self._state))
            self._needs_update = True

    @property
    def collision_objects(self):
        return self.dynamic_objects + self.static_objects

    @property
    def contacts(self):
        """Return a list of all current contacts."""
        if self._needs_update:
            pb.perform_discrete_collision_detection()
            self._contacts = self.world.get_contacts()
            self._needs_update = False
        return self._contacts

    @profile
    def closest_distances(self, query_batch):
        """Return all closest distances, given a query.

        :param query_batch: Dictionary {Object: Max distance}
        :return: {Object: [ClosestPair]}
        """
        if self._needs_update:
            self.world.update_aabbs()
            # Not 100% sure that the collision detection step is not necessary. Seems like it isn't.
            # self.world.perform_discrete_collision_detection()
            # self._needs_update = False
        return self.world.get_closest_batch(query_batch)

    def __eq__(self, other):
        if isinstance(other, CollisionSubworld):
            return self.names == other.names and self.free_symbols == other.free_symbols
        return False


def contact_geometry(pose_a, pose_b, path_a, path_b):
    """Generates a standard contact model, given two object pose expressions
    and object paths.
    """
    cont    = ContactSymbolContainer(path_a, path_b)
    normal  = gm.vector3(cont.normal_x, cont.normal_y, cont.normal_z)
    point_a = gm.dot(pose_a, gm.point3(cont.on_a_x, cont.on_a_y, cont.on_a_z))
    point_b = gm.dot(pose_b, gm.point3(cont.on_b_x, cont.on_b_y, cont.on_b_z))
    return point_a, point_b, normal

def closest_distance(pose_a, pose_b, path_a, path_b):
    """Generates a closest distance expression, given two object pose expressions
    and object paths.
    """
    point_a, point_b, _ = contact_geometry(pose_a, pose_b, path_a, path_b)
    return gm.norm(point_a - point_b)

def closest_distance_constraint(obj_a_pose, obj_b_pose, obj_a_path, obj_b_path, margin=0):
    """Generates an impenetrability constraint, given two object poses, paths, and a margin.
    """
    dist = closest_distance(obj_a_pose, obj_b_pose, obj_a_path, obj_b_path)
    return Constraint(margin - dist, 1000, dist)

def contact_constraint(obj_a_pose, obj_b_pose, obj_a_path, obj_b_path):
    """Generates a contact constraint, given two object poses, paths.
    """
    dist = closest_distance(obj_a_pose, obj_b_pose, obj_a_path, obj_b_path)
    return Constraint(-dist, -dist, dist)

def contact_geometry_world(obj_pose, obj_path):
    return contact_geometry(obj_pose, gm.eye(4), obj_path, Path('anon'))

def closest_distance_world(obj_pose, obj_path):
    return contact_distance(obj_pose, gm.eye(4), obj_path, Path('anon'))

def closest_distance_constraint_world(obj_pose, obj_path, margin=0):
    return closest_distance_constraint(obj_pose, gm.eye(4), obj_path, Path('anon'), margin)

def contact_constraint_world(obj_pose, obj_path):
    return contact_constraint(obj_pose, gm.eye(4), obj_path, Path('anon'))

def generate_contact_model(actuated_point, actuated_symbols, contact_point, contact_normal, affected_dof, dist_threshold=0.01, set_inanimate=True, default_bound=1e9):
    """Normal is assumed to point towards the actuator.
    This is a very bad and limited model. Should suffice for 1-Dof pushes though.
    """
    distance     = gm.dot_product(contact_normal, actuated_point - contact_point)
    in_contact   = gm.less_than(distance, dist_threshold)
    actuated_jac = gm.vector3(gm.get_diff(actuated_point[0], actuated_symbols),
                              gm.get_diff(actuated_point[1], actuated_symbols),
                              gm.get_diff(actuated_point[2], actuated_symbols))

    actuated_n   = gm.dot_product(actuated_point, contact_normal)
    actuated_t   = actuated_point - actuated_n * contact_normal

    contact_n    = gm.dot_product(contact_point,  contact_normal)
    contact_t    = contact_point - contact_n * contact_normal

    tangent_dist = gm.norm(contact_t - actuated_t)

    # print('distance:\n{}'.format(distance))
    # print('impenetrability lb:\n{}'.format(-distance - alg_not(in_contact) * default_bound))

    imp_lb = -distance - gm.alg_not(in_contact) * default_bound

    out = {'direction_limit': Constraint(-default_bound, 0, gm.dot_product(contact_point, contact_normal)),
           'impenetrability': Constraint(imp_lb, default_bound, distance),
           #'motion_alignment_tangent': Constraint(-alg_not(in_contact), alg_not(in_contact), tangent_dist),
           #'motion_alignment_normal': Constraint(-alg_not(in_contact), alg_not(in_contact), actuated_n - contact_n)
           }

    for s in affected_dof:
        contact_jac = gm.vector3(gm.diff(contact_point[0], s),
                                 gm.diff(contact_point[1], s),
                                 gm.diff(contact_point[2], s)) * gm.get_diff(s)
        #  out['motion_alignment_{}'.format(s)] = Constraint(-in_contact * default_bound, 0, gm.dot_product(contact_normal, contact_jac))
        if set_inanimate:
            out['inanimate_{}'.format(s)] = Constraint(-in_contact * default_bound, in_contact * default_bound, s)

    return out, in_contact


class ContactSymbolContainer(object):
    """Generates a symbolic contact model consisting of a point on object A,
    one on B, and a normal from B to A.
    """
    def __init__(self, path_obj, path_other=0):
        contact_name = ('contact',) + path_obj + (obj_to_obj_infix,) + path_other
        self.on_a_x = (contact_name + ('onA', 'x')).to_symbol()
        self.on_a_y = (contact_name + ('onA', 'y')).to_symbol()
        self.on_a_z = (contact_name + ('onA', 'z')).to_symbol()
        self.on_b_x = (contact_name + ('onB', 'x')).to_symbol()
        self.on_b_y = (contact_name + ('onB', 'y')).to_symbol()
        self.on_b_z = (contact_name + ('onB', 'z')).to_symbol()
        self.normal_x = (contact_name + ('normal', 'x')).to_symbol()
        self.normal_y = (contact_name + ('normal', 'y')).to_symbol()
        self.normal_z = (contact_name + ('normal', 'z')).to_symbol()

    def __eq__(self, other):
        if isinstance(other, ContactSymbolContainer):
            return gm.eq_expr(self.on_a_x, other.on_a_x) and \
                   gm.eq_expr(self.on_a_y, other.on_a_y) and \
                   gm.eq_expr(self.on_a_z, other.on_a_z) and \
                   gm.eq_expr(self.on_b_x, other.on_b_x) and \
                   gm.eq_expr(self.on_b_y, other.on_b_y) and \
                   gm.eq_expr(self.on_b_z, other.on_b_z) and \
                   gm.eq_expr(self.normal_x, other.normal_x) and \
                   gm.eq_expr(self.normal_y, other.normal_y) and \
                   gm.eq_expr(self.normal_z, other.normal_z)
        return False


pb_zero_vector = pb.Vector3(0,0,0)
pb_far_away_vector = pb.Vector3(1e8,1e8,-1e8)
pb_default_normal  = pb.Vector3(0,0,1)

class ContactHandler(object):
    """Helper to process the results from a contact query.
    Can differentiate between named and anonymous contacts.
    """
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
            self.handle_contact(pb_zero_vector, pb_zero_vector, pb_default_normal, other_path)

    def add_passive_handle(self):
        self.var_map[self._num_anon_contacts] = ContactSymbolContainer(self.obj_path, Path(f'anon/{self._num_anon_contacts}'))
        self.handle_contact(pb_zero_vector, pb_zero_vector, pb_default_normal, self._num_anon_contacts)
        self._num_anon_contacts += 1


    @profile
    def handle_contact(self, point_a, point_b, normal, name=0):
        if name not in self.var_map:
            raise Exception('Name {} is not known to contact handler'.format(name))
        container = self.var_map[name]
        self.state.update({container.on_a_x: point_a[0],
                           container.on_a_y: point_a[1],
                           container.on_a_z: point_a[2],
                           container.on_b_x: point_b[0],
                           container.on_b_y: point_b[1],
                           container.on_b_z: point_b[2],
                           container.normal_x: normal[0],
                           container.normal_y: normal[1],
                           container.normal_z: normal[2]})

    @profile
    def handle_contacts(self, contacts, name_resolver=None):
        anon_idx = 0
        for cp in contacts:
            if len(cp.points) > 0:
                if name_resolver is not None and cp.obj_b in name_resolver and name_resolver[cp.obj_b] in self.var_map:
                    name = name_resolver[cp.obj_b]
                    self.handle_contact(cp.points[0].point_a, cp.points[0].point_b, cp.points[0].normal_world_b, name)
                elif anon_idx < self._num_anon_contacts:
                    self.handle_contact(cp.points[0].point_a, cp.obj_b.np_transform.dot(cp.points[0].point_b), cp.points[0].normal_world_b, anon_idx)
                    anon_idx += 1
        
        if anon_idx < self._num_anon_contacts:
            for x in range(anon_idx, self._num_anon_contacts):
                self.handle_contact(pb_zero_vector, pb_far_away_vector, pb_default_normal, anon_idx)

    def __eq__(self, other):
        if isinstance(other, ContactHandler):
            return self.obj_path == other.obj_path and self.state == other.state and self.var_map == other.state
        return False
