import unittest as ut

from kineverse.gradients.gradient_math     import *
from kineverse.model.frames                import Frame, Transform
from kineverse.model.kinematic_model       import KinematicModel, Path
from kineverse.operations.frame_operations import collect_chain,           \
                                                  fk_a_in_b,               \
                                                  CreateRelativeTransform, \
                                                  CreateRelativeFrame
from kineverse.operations.basic_operations import CreateComplexObject


class TestFrames(ut.TestCase):

    def test_collect_chain(self):
        km = KinematicModel()

        f_a = Frame('map', None)
        f_b = Frame(  'a', None)
        f_c = Frame(  'b', None)
        f_d = Frame(  'c', None)
        f_e = Frame(  'b', None)

        km.set_data('a', f_a)
        km.set_data('b', f_b)
        km.set_data('c', f_c)
        km.set_data('d', f_d)
        km.set_data('e', f_e)

        chain = collect_chain(km, f_d)
        self.assertEquals(chain, [f_a, f_b, f_c, f_d])


    def test_fk_a_in_b(self):
        km = KinematicModel()
        
        p1 = translation3(1,-3, 5)
        axis1  = vector3(1, -2, 5)
        axis1 /= norm(axis1)
        p2 = rotation3_axis_angle(axis1, 1.67)
        p3 = translation3(-7, 1, 0)
        axis2  = vector3(5, 0, 3)
        axis2 /= norm(axis2)
        p4 = rotation3_axis_angle(axis2, -0.6)
        p5 = translation3(0, 0, 4)
        p6 = frame3_rpy(1.2, -0.3, 0.67, [1,2,3])

        f_a = Frame('map', p1, p1)
        f_b = Frame(  'a', f_a.pose * p2, p2)
        f_c = Frame(  'b', f_b.pose * p3, p3)
        f_d = Frame(  'c', f_c.pose * p4, p4)
        f_e = Frame(  'b', f_b.pose * p5, p5)        
        f_f = Frame('map', p6, p6)
        f_g = Frame('lol', spw.eye(4))

        km.set_data('a', f_a)
        km.set_data('b', f_b)
        km.set_data('c', f_c)
        km.set_data('d', f_d)
        km.set_data('e', f_e)
        km.set_data('f', f_f)
        km.set_data('g', f_g)

        d_in_b = fk_a_in_b(km, f_d, f_b)
        b_in_d = fk_a_in_b(km, f_b, f_d)
        d_in_e = fk_a_in_b(km, f_d, f_e)
        d_in_f = fk_a_in_b(km, f_d, f_f)

        self.assertEquals(d_in_b, f_c.to_parent * f_d.to_parent)
        self.assertEquals(b_in_d, inverse_frame(f_c.to_parent * f_d.to_parent))
        self.assertEquals(d_in_e, inverse_frame(f_e.to_parent) * f_c.to_parent * f_d.to_parent)
        self.assertEquals(d_in_f, inverse_frame(f_f.pose) * f_d.pose)

        with self.assertRaises(Exception):
            fk_a_in_b(km, f_d, f_g)


    def test_create_relative_frame_and_transform(self):
        km = KinematicModel()
        
        p1 = translation3(1,-3, 5)
        axis1  = vector3(1, -2, 5)
        axis1 /= norm(axis1)
        p2 = rotation3_axis_angle(axis1, 1.67)
        p3 = translation3(-7, 1, 0)
        axis2  = vector3(5, 0, 3)
        axis2 /= norm(axis2)
        p4 = rotation3_axis_angle(axis2, -0.6)
        p5 = translation3(0, 0, 4)
        p6 = frame3_rpy(1.2, -0.3, 0.67, [1,2,3])

        f_a  = Frame('map', p1, p1)
        f_b  = Frame(  'a', f_a.pose * p2, p2)
        f_c  = Frame(  'b', f_b.pose * p3, p3)
        f_c2 = Frame(  'b', f_b.pose * p6, p6)
        f_d  = Frame(  'c', f_c.pose * p4, p4)
        f_e  = Frame(  'b', f_b.pose * p5, p5)        
        f_f  = Frame('map', p6, p6)
        f_g  = Frame('lol', spw.eye(4))


        km.apply_operation('create a', CreateComplexObject(Path('a'), f_a))
        km.apply_operation('create b', CreateRelativeFrame(Path('b'), f_b))
        km.apply_operation('create c', CreateRelativeFrame(Path('c'), f_c))
        km.apply_operation('create d', CreateRelativeFrame(Path('d'), f_d))
        km.apply_operation('create e', CreateRelativeFrame(Path('e'), f_e))
        km.apply_operation('create f', CreateComplexObject(Path('f'), f_f))
        km.apply_operation('create g', CreateComplexObject(Path('g'), f_g))

        d_in_b = fk_a_in_b(km, f_d, f_b)
        b_in_d = fk_a_in_b(km, f_b, f_d)
        d_in_e = fk_a_in_b(km, f_d, f_e)
        d_in_f = fk_a_in_b(km, f_d, f_f)

        km.apply_operation('tf d to e', CreateRelativeTransform(Path('d_to_e'), Path('d'), Path('e')))
        self.assertTrue(km.has_data('d_to_e'))
        tf = km.get_data('d_to_e')
        self.assertEquals(tf.from_frame, 'd')
        self.assertEquals(tf.to_frame,   'e')
        self.assertEquals(tf.pose,    d_in_e)

        km.apply_operation('create c', CreateRelativeFrame(Path('c'), f_c2))
        km.clean_structure()
        tf = km.get_data('d_to_e')
        self.assertEquals(tf.pose, inverse_frame(f_e.to_parent) * f_c2.to_parent * f_d.to_parent)


if __name__ == '__main__':
    ut.main()
