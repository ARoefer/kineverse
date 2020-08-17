import unittest as ut

import kineverse.gradients.gradient_math as gm

from kineverse.utils                       import res_pkg_path
from kineverse.operations.basic_operations import CreateValue
from kineverse.model.articulation_model    import ArticulationModel, Path
from kineverse.operations.urdf_operations  import load_urdf,         \
                                                  RigidBody,         \
                                                  FixedJoint,        \
                                                  PrismaticJoint,    \
                                                  RevoluteJoint,     \
                                                  ContinuousJoint,   \
                                                  CreateURDFFrameConnection
from kineverse.visualization.graph_generator import generate_modifications_graph, \
                                                    generate_dependency_graph, \
                                                    plot_graph
from urdf_parser_py.urdf import URDF


class TestURDF(ut.TestCase):
    
    def test_model_eq(self):
        urdf_model = URDF.from_xml_file(res_pkg_path('package://kineverse/urdf/testbot.urdf'))
        ks1 = ArticulationModel()
        ks2 = ArticulationModel()
        load_urdf(ks1, Path(urdf_model.name), urdf_model)
        load_urdf(ks2, Path(urdf_model.name), urdf_model)
        self.assertEquals(ks1, ks2)

    def test_load(self):
        urdf_model = URDF.from_xml_file(res_pkg_path('package://kineverse/urdf/testbot.urdf'))
        km = ArticulationModel()
        load_urdf(km, Path(urdf_model.name), urdf_model)

    def test_double_reload(self):
        km = ArticulationModel()
        urdf_model = URDF.from_xml_file(res_pkg_path('package://kineverse/urdf/testbot.urdf'))
        load_urdf(km, Path(urdf_model.name), urdf_model)
        km.clean_structure()
        eef_frame_1 = km.get_data(Path(urdf_model.name) + Path('links/gripper_link/pose'))

        load_urdf(km, Path(urdf_model.name), urdf_model)
        km.clean_structure()
        eef_frame_2 = km.get_data(Path(urdf_model.name) + Path('links/gripper_link/pose'))

        self.assertTrue(gm.cm.numeric_eq(eef_frame_1, eef_frame_2))


    def test_fixed_joint(self):
        km = ArticulationModel()

        a  = gm.Position('a')
        b  = gm.Position('b')
        parent_pose     = gm.frame3_axis_angle(gm.vector3(0,1,0), a, gm.point3(0, b, 5))
        joint_transform = gm.translation3(7, -5, 33)
        child_pose      = gm.dot(parent_pose, joint_transform)

        km.apply_operation('create parent', CreateValue(Path('parent'), RigidBody('', parent_pose)))
        km.apply_operation('create child', CreateValue(Path('child'),  RigidBody('', gm.eye(4))))
        self.assertTrue(km.has_data('parent/pose'))
        self.assertTrue(km.has_data('child/pose'))

        joint = FixedJoint('parent', 'child', joint_transform)
        km.apply_operation('create fixed_joint', CreateValue(Path('fixed_joint'), joint))
        km.apply_operation('connect parent child', CreateURDFFrameConnection(Path('fixed_joint'), 
                                                                             Path('parent'), 
                                                                             Path('child')))
        self.assertTrue(km.has_data('fixed_joint'))
        self.assertTrue(gm.cm.numeric_eq(km.get_data('child/pose'), child_pose))
        self.assertEquals(km.get_data('child/parent_joint'), 'fixed_joint')

    def test_prismatic_joint(self):
        km = ArticulationModel()

        a  = gm.Position('a')
        b  = gm.Position('b')
        c  = gm.Position('c')
        parent_pose     = gm.frame3_axis_angle(gm.vector3(0,1,0), a, gm.point3(0, b, 5))
        joint_transform = gm.translation3(7, -5, 33)
        axis            = gm.vector3(1, -3, 7)
        position        = c
        pos_expr        = axis * position
        child_pose      = gm.dot(parent_pose, joint_transform, gm.translation3(pos_expr[0], 
                                                                               pos_expr[1], 
                                                                               pos_expr[2]))

        joint = PrismaticJoint('parent', 'child', position, axis, joint_transform, -1, 2, 0.5)

        km.apply_operation('create parent', CreateValue(Path('parent'), RigidBody('', parent_pose)))
        km.apply_operation('create child', CreateValue(Path('child'),  RigidBody('', gm.eye(4))))
        self.assertTrue(km.has_data('parent/pose'))
        self.assertTrue(km.has_data('child/pose'))

        km.apply_operation('create prismatic joint', CreateValue(Path('prismatic_joint'), joint))
        self.assertTrue(km.has_data('prismatic_joint'))

        km.apply_operation('connect parent child',  CreateURDFFrameConnection(Path('prismatic_joint'),
                                                                              Path('parent'),
                                                                              Path('child')))
        self.assertTrue(gm.cm.numeric_eq(km.get_data('child/pose'), child_pose))
        self.assertEquals(km.get_data('child/parent_joint'), 'prismatic_joint')

    def test_revolute_and_continuous_joint(self):
        km = ArticulationModel()

        a  = gm.Position('a')
        b  = gm.Position('b')
        c  = gm.Position('c')
        parent_pose     = gm.frame3_axis_angle(gm.vector3(0,1,0), a, gm.point3(0, b, 5))
        joint_transform = gm.translation3(7, -5, 33)
        axis            = gm.vector3(1, -3, 7)
        axis            = axis / gm.norm(axis)
        position        = c
        child_pose      = gm.dot(parent_pose, joint_transform, gm.rotation3_axis_angle(axis, position))

        km.apply_operation('create parent', CreateValue(Path('parent'), RigidBody('', parent_pose)))
        km.apply_operation('create child', CreateValue(Path('child'),  RigidBody('', gm.eye(4))))
        self.assertTrue(km.has_data('parent/pose'))
        self.assertTrue(km.has_data('child/pose'))

        joint = RevoluteJoint('parent', 'child', position, axis, joint_transform, -1, 2, 0.5)
        km.apply_operation('create revolute joint', CreateValue(Path('revolute_joint'), joint))
        self.assertTrue(km.has_data('revolute_joint'))

        km.apply_operation('connect parent child', 
                           CreateURDFFrameConnection(Path('revolute_joint'),
                                                     Path('parent'), 
                                                     Path('child')))
        self.assertTrue(gm.cm.numeric_eq(km.get_data('child/pose'), child_pose))
        self.assertEquals(km.get_data('child/parent_joint'), 'revolute_joint')

        km.remove_operation('connect parent child')
        km.remove_operation('create revolute joint')

        joint = ContinuousJoint('parent', 'child', position, axis, joint_transform)
        km.apply_operation('create continuous joint', CreateValue(Path('continuous_joint'), joint))
        self.assertTrue(km.has_data('continuous_joint'))
        self.assertFalse(km.has_data('revolute_joint'))
        km.apply_operation('connect parent child',
                           CreateURDFFrameConnection(Path('continuous_joint'),
                                                     Path('parent'), 
                                                     Path('child')))
        self.assertTrue(gm.cm.numeric_eq(km.get_data('child/pose'), child_pose))
        self.assertEquals(km.get_data('child/parent_joint'), 'continuous_joint')

    def test_model_reform(self):
        km = ArticulationModel()

        a  = gm.Position('a')
        b  = gm.Position('b')
        c  = gm.Position('c')
        parent_pose_a   = gm.frame3_axis_angle(gm.vector3(0,1,0), a, gm.point3(    0, b, 5))
        parent_pose_b   = gm.frame3_axis_angle(gm.vector3(1,0,0), b, gm.point3(7 * b, 0, 5))
        joint_transform = gm.translation3(7, -5, 33)
        axis            = gm.vector3(1, -3, 7)
        axis            = axis
        position        = c
        pos_expr        = axis * position
        child_pose_a    = gm.dot(parent_pose_a, joint_transform, gm.translation3(pos_expr[0], 
                                                                                 pos_expr[1], 
                                                                                 pos_expr[2]))
        child_pose_b    = gm.dot(parent_pose_b, joint_transform, gm.translation3(pos_expr[0], 
                                                                                 pos_expr[1], 
                                                                                 pos_expr[2]))

        km.apply_operation('create parent', CreateValue(Path('parent'), RigidBody('', parent_pose_a)))
        km.apply_operation('create child', CreateValue(Path('child'),  RigidBody('', gm.eye(4))))
        self.assertTrue(km.has_data('parent/pose'))
        self.assertTrue(km.has_data('child/pose'))

        joint = PrismaticJoint('parent', 'child', position, axis, joint_transform, -1, 2, 0.5)
        km.apply_operation('create joint', CreateValue(Path('joint'), joint))
        self.assertTrue(km.has_data('joint'))

        km.apply_operation('connect parent child', 
                           CreateURDFFrameConnection(Path('joint'), Path('parent'), Path('child')))
        self.assertTrue(gm.cm.numeric_eq(km.get_data('child/pose'), child_pose_a))
        km.apply_operation('create parent', CreateValue(Path('parent'), RigidBody('', parent_pose_b)))
        # plot_graph(generate_dependency_graph(km), 'test_model_reform_dep.png')
        # plot_graph(generate_modifications_graph(km), 'test_model_reform_mod.png')
        km.clean_structure()
        self.assertTrue(km.has_data('joint'))
        self.assertTrue(gm.cm.numeric_eq(km.get_data('child/pose'), child_pose_b, verbose=1))




if __name__ == '__main__':
    ut.main()
