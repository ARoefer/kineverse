import unittest as ut

from kineverse.utils                       import res_pkg_path
from kineverse.operations.basic_operations import CreateComplexObject
from kineverse.gradients.gradient_math     import translation3, \
                                                  rotation3_axis_angle, \
                                                  frame3_axis_angle, \
                                                  vector3, \
                                                  point3, \
                                                  norm, \
                                                  spw
from kineverse.model.kinematic_model       import KinematicModel, Path
from kineverse.operations.urdf_operations  import load_urdf,         \
                                                  KinematicLink,     \
                                                  SetFixedJoint,     \
                                                  SetPrismaticJoint, \
                                                  SetRevoluteJoint,  \
                                                  SetContinuousJoint 

from urdf_parser_py.urdf import URDF


class TestURDF(ut.TestCase):
    
    def test_load(self):
        urdf_model = URDF.from_xml_file(res_pkg_path('package://kineverse/urdf/testbot.urdf'))
        ks = KinematicModel()
        load_urdf(ks, Path(urdf_model.name), urdf_model)

    def test_double_reload(self):
        km = KinematicModel()
        urdf_model = URDF.from_xml_file(res_pkg_path('package://kineverse/urdf/testbot.urdf'))
        load_urdf(km, Path(urdf_model.name), urdf_model)
        km.clean_structure()
        eef_frame_1 = km.get_data(Path(urdf_model.name) + Path('links/gripper_link/pose'))

        load_urdf(km, Path(urdf_model.name), urdf_model)
        km.clean_structure()
        eef_frame_2 = km.get_data(Path(urdf_model.name) + Path('links/gripper_link/pose'))

        self.assertEquals(eef_frame_1, eef_frame_2)


    def test_fixed_joint(self):
        ks = KinematicModel()

        a  = spw.Symbol('a_p')
        b  = spw.Symbol('b_p')
        parent_pose     = frame3_axis_angle(vector3(0,1,0), a, point3(0, b, 5))
        joint_transform = translation3(7, -5, 33)
        child_pose      = parent_pose * joint_transform

        ks.apply_operation('create parent', CreateComplexObject(Path('parent'), KinematicLink('', parent_pose)))
        ks.apply_operation('create child', CreateComplexObject(Path('child'),  KinematicLink('', spw.eye(4))))
        self.assertTrue(ks.has_data('parent/pose'))
        self.assertTrue(ks.has_data('child/pose'))

        ks.apply_operation('connect parent child', SetFixedJoint(Path('parent/pose'), Path('child/pose'), Path('fixed_joint'), joint_transform))
        self.assertTrue(ks.has_data('fixed_joint'))
        self.assertEquals(ks.get_data('child/pose'), child_pose)

    def test_prismatic_joint(self):
        ks = KinematicModel()

        a  = spw.Symbol('a_p')
        b  = spw.Symbol('b_p')
        c  = spw.Symbol('c_p')
        parent_pose     = frame3_axis_angle(vector3(0,1,0), a, point3(0, b, 5))
        joint_transform = translation3(7, -5, 33)
        axis            = vector3(1, -3, 7)
        position        = c
        child_pose      = parent_pose * joint_transform * translation3(*(axis[:,:3] * position))

        ks.apply_operation('create parent', CreateComplexObject(Path('parent'), KinematicLink('', parent_pose)))
        ks.apply_operation('create child', CreateComplexObject(Path('child'),  KinematicLink('', spw.eye(4))))
        self.assertTrue(ks.has_data('parent/pose'))
        self.assertTrue(ks.has_data('child/pose'))

        ks.apply_operation('connect parent child', 
                           SetPrismaticJoint(Path('parent/pose'), 
                                             Path('child/pose'), 
                                             Path('fixed_joint'), 
                                             joint_transform,
                                             axis,
                                             position,
                                             -1, 2, 0.5))
        self.assertTrue(ks.has_data('fixed_joint'))
        self.assertEquals(ks.get_data('child/pose'), child_pose)

    def test_revolute_and_continuous_joint(self):
        ks = KinematicModel()

        a  = spw.Symbol('a_p')
        b  = spw.Symbol('b_p')
        c  = spw.Symbol('c_p')
        parent_pose     = frame3_axis_angle(vector3(0,1,0), a, point3(0, b, 5))
        joint_transform = translation3(7, -5, 33)
        axis            = vector3(1, -3, 7)
        axis            = axis / norm(axis)
        position        = c
        child_pose      = parent_pose * joint_transform * rotation3_axis_angle(axis, position)

        ks.apply_operation('create parent', CreateComplexObject(Path('parent'), KinematicLink('', parent_pose)))
        ks.apply_operation('create child', CreateComplexObject(Path('child'),  KinematicLink('', spw.eye(4))))
        self.assertTrue(ks.has_data('parent/pose'))
        self.assertTrue(ks.has_data('child/pose'))

        ks.apply_operation('connect parent child', 
                           SetRevoluteJoint(Path('parent/pose'), 
                                            Path('child/pose'), 
                                            Path('fixed_joint'), 
                                            joint_transform,
                                            axis,
                                            position,
                                            -1, 2, 0.5))
        self.assertTrue(ks.has_data('fixed_joint'))
        self.assertEquals(ks.get_data('child/pose'), child_pose)
        ks.remove_operation('connect parent child')
        ks.apply_operation('connect parent child',
                           SetContinuousJoint(Path('parent/pose'), 
                                              Path('child/pose'), 
                                              Path('fixed_joint'), 
                                              joint_transform,
                                              axis,
                                              position, 0.5))

    def test_model_reform(self):
        ks = KinematicModel()

        a  = spw.Symbol('a_p')
        b  = spw.Symbol('b_p')
        c  = spw.Symbol('c_p')
        parent_pose_a   = frame3_axis_angle(vector3(0,1,0), a, point3(0, b, 5))
        parent_pose_b   = frame3_axis_angle(vector3(1,0,0), b, point3(7* b, 0, 5))
        joint_transform = translation3(7, -5, 33)
        axis            = vector3(1, -3, 7)
        axis            = axis
        position        = c
        child_pose_a    = parent_pose_a * joint_transform * translation3(*(axis[:,:3] * position))
        child_pose_b    = parent_pose_b * joint_transform * translation3(*(axis[:,:3] * position))

        ks.apply_operation('create parent', CreateComplexObject(Path('parent'), KinematicLink('', parent_pose_a)))
        ks.apply_operation('create child', CreateComplexObject(Path('child'),  KinematicLink('', spw.eye(4))))
        self.assertTrue(ks.has_data('parent/pose'))
        self.assertTrue(ks.has_data('child/pose'))

        ks.apply_operation('connect parent child', 
                           SetPrismaticJoint(Path('parent/pose'), 
                                             Path('child/pose'), 
                                             Path('fixed_joint'), 
                                             joint_transform,
                                             axis,
                                             position,
                                             -1, 2, 0.5))
        self.assertTrue(ks.has_data('fixed_joint'))
        self.assertEquals(ks.get_data('child/pose'), child_pose_a)
        ks.apply_operation('create parent', CreateComplexObject(Path('parent'), KinematicLink('', parent_pose_b)))
        ks.clean_structure()
        self.assertTrue(ks.has_data('fixed_joint'))
        self.assertEquals(ks.get_data('child/pose'), child_pose_b)        




if __name__ == '__main__':
    ut.main()