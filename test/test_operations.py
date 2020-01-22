import unittest as ut

from kineverse.utils import bb
from kineverse.operations.basic_operations import *
from kineverse.gradients.gradient_math     import *
from kineverse.model.kinematic_model       import KinematicModel


class TestOperations(ut.TestCase):
    def test_add_single(self):
        ks = KinematicModel()
        p  = Path('my_var')
        op = CreateSingleValue(p, 5)
        ks.apply_operation('create my_var', op)
        self.assertTrue(ks.has_data(p))
        self.assertEquals(ks.get_data(p), 5)
        ks.remove_operation('create my_var')
        self.assertFalse(ks.has_data(p))


    def test_add_object(self):
        ks  = KinematicModel()
        obj = bb(some_str='lol', some_scalar=7.5, some_subobj=bb(x=4, y=5, z=10))
        op  = CreateComplexObject(Path('my_obj'), obj)
        self.assertIn('path/some_str', op.mod_paths)
        self.assertIn('path/some_scalar', op.mod_paths)
        self.assertIn('path/some_subobj', op.mod_paths)
        self.assertIn('path/some_subobj/x', op.mod_paths)
        self.assertIn('path/some_subobj/y', op.mod_paths)
        self.assertIn('path/some_subobj/z', op.mod_paths)
        self.assertIn('path', op._root_set)
        self.assertEquals(op.mod_paths['path/some_str'],     ('my_obj','some_str'))
        self.assertEquals(op.mod_paths['path/some_scalar'],  ('my_obj','some_scalar'))
        self.assertEquals(op.mod_paths['path/some_subobj'],  ('my_obj','some_subobj'))
        self.assertEquals(op.mod_paths['path/some_subobj/x'],('my_obj','some_subobj','x'))
        self.assertEquals(op.mod_paths['path/some_subobj/y'],('my_obj','some_subobj','y'))
        self.assertEquals(op.mod_paths['path/some_subobj/z'],('my_obj','some_subobj','z'))
        self.assertEquals(op._root_set['path'], ('my_obj'))
        ks.apply_operation('create my_obj', op)
        self.assertTrue(ks.has_data('my_obj'))
        self.assertEquals(ks.get_data('my_obj/some_str'), 'lol')
        self.assertEquals(ks.get_data('my_obj/some_scalar'), 7.5)
        self.assertEquals(ks.get_data('my_obj/some_subobj/x'), 4)
        self.assertEquals(ks.get_data('my_obj/some_subobj/y'), 5)
        self.assertEquals(ks.get_data('my_obj/some_subobj/z'), 10)
        ks.remove_operation('create my_obj')
        self.assertFalse(ks.has_data('my_obj'))
        self.assertFalse(ks.has_data('my_obj/some_scalar'))
        self.assertFalse(ks.has_data('my_obj/some_subobj'))
        self.assertFalse(ks.has_data('my_obj/some_subobj/x'))
        self.assertFalse(ks.has_data('my_obj/some_subobj/y'))
        self.assertFalse(ks.has_data('my_obj/some_subobj/z'))

    def test_add_fn_call(self):
        ks  = KinematicModel()
        ks.apply_operation('create my_var', CreateSingleValue('my_var', 5))
        ks.apply_operation('create vec_a',  CreateSingleValue('vec_a', spw.vector3(1,0,0)))
        ks.apply_operation('create vec_b',  CreateSingleValue('vec_b', spw.vector3(0,1,0)))

        with self.assertRaises(Exception):
            op = CallFunctionOperator('sin_of_my_var', sin)

        op = CallFunctionOperator('sin_of_my_var', sin, Path('my_var'))
        self.assertIn('expr', op.args_paths)
        self.assertEquals(op.args_paths['expr'], 'my_var')
        ks.apply_operation('compute sin of my_var', op)
        self.assertTrue(ks.has_data('sin_of_my_var'))
        self.assertEquals(ks.get_data('sin_of_my_var'), sin(5))
        ks.remove_operation('compute sin of my_var')
        self.assertFalse(ks.has_data('sin_of_my_var'))

        with self.assertRaises(Exception):
            op = CallFunctionOperator('cross_of_a_b', cross, Path('vec_a'))

        op = CallFunctionOperator('cross_of_a_b', cross, Path('vec_a'), Path('vec_b'))
        self.assertIn('u', op.args_paths)
        self.assertEquals(op.args_paths['u'], 'vec_a')
        self.assertIn('v', op.args_paths)
        self.assertEquals(op.args_paths['v'], 'vec_b')
        ks.apply_operation('compute cross of vectors', op)
        self.assertTrue(ks.has_data('cross_of_a_b'))
        self.assertEquals(ks.get_data('cross_of_a_b'), spw.vector3(0,0,1))
        ks.remove_operation('compute cross of vectors')
        self.assertFalse(ks.has_data('cross_of_a_b'))

if __name__ == '__main__':
    ut.main()