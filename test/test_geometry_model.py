import kineverse.gradients.gradient_math as gm
import unittest as ut
import betterpybullet as pb

from kineverse.operations.basic_operations import CreateValue
from kineverse.gradients.diff_logic       import Position
from kineverse.gradients.gradient_math    import *
from kineverse.model.paths                import Path, CPath
from kineverse.model.geometry_model       import GeometryModel,\
                                                 CollisionSubworld,\
                                                 ArticulatedObject,\
                                                 RigidBody, Geometry
from kineverse.operations.urdf_operations import PrismaticJoint, RevoluteJoint

class TestGeometryModel(ut.TestCase):

    def test_link_insertion(self):
        km = GeometryModel()

        sym_a = Position('a')
        sym_b = Position('b')
        sym_c = Position('c')

        box_shape  = Geometry(CPath('my_box'),  gm.eye(4), 'box')
        mesh_shape = Geometry(CPath('my_mesh'), gm.eye(4), 'mesh', mesh='package://kineverse/meshes/suzanne.obj')

        box_link  = RigidBody(CPath('world'), translation3(sym_a, 1, 0), geometry={'0' : box_shape}, collision={'0' : box_shape})
        mesh_link = RigidBody(CPath('world'), translation3(1, sym_b, 0), geometry={'0' : mesh_shape}, collision={'0' : mesh_shape}) 

        km.apply_operation('create my_box',  CreateValue(Path('my_box'),  box_link))
        km.apply_operation('create my_mesh', CreateValue(Path('my_mesh'), mesh_link))

        km.clean_structure()
        km.dispatch_events() # Generates the pose expressions for links

        sub_world = km.get_active_geometry({sym_a})
        self.assertEqual(len(sub_world.names), 1)
        self.assertIn(Path('my_box'), sub_world.names)
        self.assertIn(Path('my_box'), sub_world.named_objects)

        sub_world = km.get_active_geometry({sym_a, sym_b})
        self.assertEqual(len(sub_world.names), 2)
        self.assertIn(Path('my_box'), sub_world.names)
        self.assertIn(Path('my_mesh'), sub_world.names)
        self.assertIn(Path('my_box'),  sub_world.named_objects)
        self.assertIn(Path('my_mesh'), sub_world.named_objects)

        sub_world = km.get_active_geometry({sym_c})
        self.assertEqual(len(sub_world.names), 0)


    def test_urdf_insertion(self):
        km = GeometryModel()

        sym_a = Position('a')
        sym_b = Position('b')
        sym_c = Position('c')

        box_shape  = Geometry('my_box',  gm.eye(4), 'box')
        mesh_shape = Geometry('my_mesh', gm.eye(4), 'mesh', mesh='package://kineverse/meshes/suzanne.obj')

        box_link  = RigidBody('world', translation3(sym_a, 1, 0), geometry={'0' : box_shape}, collision={'0' : box_shape})
        mesh_link = RigidBody('world', translation3(1, sym_b, 0), geometry={'0' : mesh_shape}, collision={'0' : mesh_shape}) 

        robot = ArticulatedObject('my_bot')
        robot.links['my_box']  = box_link
        robot.links['my_mesh'] = mesh_link

        km.apply_operation('create my_bot', CreateValue(Path('my_bot'), robot))
        km.clean_structure()
        km.dispatch_events() # Generates the pose expressions for links        

        sub_world = km.get_active_geometry({sym_a})
        self.assertEqual(len(sub_world.names), 1)
        self.assertIn(Path('my_bot/links/my_box'), sub_world.names)
        self.assertIn(Path('my_bot/links/my_box'),  sub_world.named_objects)

        sub_world = km.get_active_geometry({sym_a, sym_b})
        self.assertEqual(len(sub_world.names), 2)
        self.assertIn(Path('my_bot/links/my_box'),  sub_world.names)
        self.assertIn(Path('my_bot/links/my_mesh'), sub_world.names)
        self.assertIn(Path('my_bot/links/my_box'),  sub_world.named_objects)
        self.assertIn(Path('my_bot/links/my_mesh'), sub_world.named_objects)

        sub_world = km.get_active_geometry({sym_c})
        self.assertEqual(len(sub_world.names), 0)


if __name__ == '__main__':
    ut.main()
