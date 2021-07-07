import unittest as ut

import kineverse.gradients.gradient_math as gm

from kineverse.model.frames                import Frame, Transform
from kineverse.model.articulation_model    import ArticulationModel, Path
from kineverse.operations.frame_operations import collect_chain,           \
                                                  fk_a_in_b,               \
                                                  CreateRelativeTransform, \
                                                  CreateRelativeFrame
from kineverse.operations.basic_operations import CreateValue


class TestFrames(ut.TestCase):

    def test_collect_chain(self):
        km = ArticulationModel()

        f_a = Frame('world', None)
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
        self.assertEqual(chain, [f_a, f_b, f_c, f_d])


    def test_fk_a_in_b(self):
        km = ArticulationModel()
        
        p1 = gm.translation3(1,-3, 5)
        axis1  = gm.vector3(1, -2, 5)
        axis1 /= gm.norm(axis1)
        p2 = gm.rotation3_axis_angle(axis1, 1.67)
        p3 = gm.translation3(-7, 1, 0)
        axis2  = gm.vector3(5, 0, 3)
        axis2 /= gm.norm(axis2)
        p4 = gm.rotation3_axis_angle(axis2, -0.6)
        p5 = gm.translation3(0, 0, 4)
        p6 = gm.frame3_rpy(1.2, -0.3, 0.67, [1,2,3])

        f_a = Frame('world', p1, p1)
        f_b = Frame(  'a', gm.dot(f_a.pose, p2), p2)
        f_c = Frame(  'b', gm.dot(f_b.pose, p3), p3)
        f_d = Frame(  'c', gm.dot(f_c.pose, p4), p4)
        f_e = Frame(  'b', gm.dot(f_b.pose, p5), p5)        
        f_f = Frame('world', p6, p6)
        f_g = Frame('lol', gm.eye(4))

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

        self.assertTrue(gm.numeric_eq(d_in_b, 
                                      gm.dot(f_c.to_parent, f_d.to_parent),
                                      default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(b_in_d,
                                      gm.inverse_frame(gm.dot(f_c.to_parent, f_d.to_parent)),
                                      default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(d_in_e,
                                      gm.dot(gm.inverse_frame(f_e.to_parent), gm.dot(f_c.to_parent, f_d.to_parent)),
                                        default_range=[-10, 10]))
        self.assertTrue(gm.numeric_eq(d_in_f,
                                      gm.dot(gm.inverse_frame(f_f.pose), f_d.pose),
                                      default_range=[-10, 10]))

        with self.assertRaises(Exception):
            fk_a_in_b(km, f_d, f_g)


    def test_create_relative_frame_and_transform(self):
        km = ArticulationModel()
        
        p1 = gm.translation3(1,-3, 5)
        axis1  = gm.vector3(1, -2, 5)
        axis1 /= gm.norm(axis1)
        p2 = gm.rotation3_axis_angle(axis1, 1.67)
        p3 = gm.translation3(-7, 1, 0)
        axis2  = gm.vector3(5, 0, 3)
        axis2 /= gm.norm(axis2)
        p4 = gm.rotation3_axis_angle(axis2, -0.6)
        p5 = gm.translation3(0, 0, 4)
        p6 = gm.frame3_rpy(1.2, -0.3, 0.67, [1,2,3])

        f_a  = Frame('world', p1, p1)
        f_b  = Frame(  'a', gm.dot(f_a.pose, p2), p2)
        f_c  = Frame(  'b', gm.dot(f_b.pose, p3), p3)
        f_c2 = Frame(  'b', gm.dot(f_b.pose, p6), p6)
        f_d  = Frame(  'c', gm.dot(f_c.pose, p4), p4)
        f_e  = Frame(  'b', gm.dot(f_b.pose, p5), p5)        
        f_f  = Frame('world', p6, p6)
        f_g  = Frame('lol', gm.eye(4))


        km.apply_operation('create a', CreateValue(Path('a'), f_a))
        km.apply_operation('create b', CreateRelativeFrame(Path('b'), Path(f_b.parent), f_b))
        km.apply_operation('create c', CreateRelativeFrame(Path('c'), Path(f_c.parent), f_c))
        km.apply_operation('create d', CreateRelativeFrame(Path('d'), Path(f_d.parent), f_d))
        km.apply_operation('create e', CreateRelativeFrame(Path('e'), Path(f_e.parent), f_e))
        km.apply_operation('create f', CreateValue(Path('f'), f_f))
        km.apply_operation('create g', CreateValue(Path('g'), f_g))

        d_in_b = fk_a_in_b(km, f_d, f_b)
        b_in_d = fk_a_in_b(km, f_b, f_d)
        d_in_e = fk_a_in_b(km, f_d, f_e)
        d_in_f = fk_a_in_b(km, f_d, f_f)

        km.apply_operation('tf d to e', CreateRelativeTransform(Path('d_to_e'), Path('d'), Path('e')))
        self.assertTrue(km.has_data('d_to_e'))
        tf = km.get_data('d_to_e')
        self.assertEqual(tf.from_frame, 'd')
        self.assertEqual(tf.to_frame,   'e')
        self.assertTrue(gm.numeric_eq(tf.pose, d_in_e, default_range=[-10, 10]))

        km.apply_operation('create c', CreateRelativeFrame(Path('c'), Path(f_c2.parent), f_c2))
        km.clean_structure()
        tf = km.get_data('d_to_e')
        self.assertTrue(gm.numeric_eq(tf.pose, gm.dot(gm.inverse_frame(f_e.to_parent), gm.dot(f_c2.to_parent, f_d.to_parent)), default_range=[-10, 10]))


if __name__ == '__main__':
    ut.main()
