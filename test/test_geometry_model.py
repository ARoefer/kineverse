import unittest as ut
import betterpybullet as pb

from kineverse.gradients.diff_logic       import create_pos
from kineverse.gradients.gradient_math    import *
from kineverse.model.paths                import Path
from kineverse.model.geometry_model       import GeometryModel, CollisionSubworld
from kineverse.operations.urdf_operations import KinematicLink, Geometry, URDFRobot, PrismaticJoint, RevoluteJoint

class TestGeometryModel(ut.TestCase):

    def test_link_insertion(self):
        gm = GeometryModel()

        sym_a = create_pos('a')
        sym_b = create_pos('b')
        sym_c = create_pos('c')

        box_shape  = Geometry('my_box', spw.eye(4), 'box')
        mesh_shape = Geometry('my_mesh', spw.eye(4), 'mesh', mesh='package://kineverse/meshes/suzanne.obj')

        box_link  = KinematicLink('map', translation3(sym_a, 1, 0), {'0' : box_shape}, {'0' : box_shape})
        mesh_link = KinematicLink('map', translation3(1, sym_b, 0), {'0' : mesh_shape}, {'0' : mesh_shape}) 

        gm.set_data('my_box', box_link)
        gm.set_data('my_mesh', mesh_link)
        gm.clean_structure()
        gm.dispatch_events() # Generates the pose expressions for links

        sub_world = gm.get_active_geometry({sym_a})
        self.assertEquals(len(sub_world.names), 1)
        self.assertIn('my_box', sub_world.names)
        self.assertIn('my_box',  sub_world.named_objects)

        sub_world = gm.get_active_geometry({sym_a, sym_b})
        self.assertEquals(len(sub_world.names), 2)
        self.assertIn('my_box', sub_world.names)
        self.assertIn('my_mesh', sub_world.names)
        self.assertIn('my_box',  sub_world.named_objects)
        self.assertIn('my_mesh', sub_world.named_objects)

        sub_world = gm.get_active_geometry({sym_c})
        self.assertEquals(len(sub_world.names), 0)


    def test_urdf_insertion(self):
        gm = GeometryModel()

        sym_a = create_pos('a')
        sym_b = create_pos('b')
        sym_c = create_pos('c')

        box_shape  = Geometry('my_box', spw.eye(4), 'box')
        mesh_shape = Geometry('my_mesh', spw.eye(4), 'mesh', mesh='package://kineverse/meshes/suzanne.obj')

        box_link  = KinematicLink('map', translation3(sym_a, 1, 0), {'0' : box_shape}, {'0' : box_shape})
        mesh_link = KinematicLink('map', translation3(1, sym_b, 0), {'0' : mesh_shape}, {'0' : mesh_shape}) 

        robot = URDFRobot('my_bot')
        robot.links['my_box']  = box_link
        robot.links['my_mesh'] = mesh_link

        gm.set_data('my_bot', robot)
        gm.clean_structure()
        gm.dispatch_events() # Generates the pose expressions for links        

        sub_world = gm.get_active_geometry({sym_a})
        self.assertEquals(len(sub_world.names), 1)
        self.assertIn('my_bot/links/my_box', sub_world.names)
        self.assertIn('my_bot/links/my_box',  sub_world.named_objects)

        sub_world = gm.get_active_geometry({sym_a, sym_b})
        self.assertEquals(len(sub_world.names), 2)
        self.assertIn('my_bot/links/my_box',  sub_world.names)
        self.assertIn('my_bot/links/my_mesh', sub_world.names)
        self.assertIn('my_bot/links/my_box',  sub_world.named_objects)
        self.assertIn('my_bot/links/my_mesh', sub_world.named_objects)

        sub_world = gm.get_active_geometry({sym_c})
        self.assertEquals(len(sub_world.names), 0)


if __name__ == '__main__':
    ut.main()