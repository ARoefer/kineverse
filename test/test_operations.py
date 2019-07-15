import unittest as ut

from kineverse.utils import bb
from kineverse.operations.basic_operations import *
from kineverse.gradients.gradient_math     import *
from kineverse.model.kinematic_state       import KinematicState


class TestOperations(ut.TestCase):
    def test_add_single(self):
        ks = KinematicState()
        p  = Path('my_var')
        op = CreateSingleValue(p, 5)
        ks.apply_operation(op, 'create my_var')
        self.assertTrue(ks.has_data(p))
        self.assertEquals(ks.get_data(p), 5)
        ks.remove_operation('create my_var')
        self.assertFalse(ks.has_data(p))


    def test_add_object(self):
        ks  = KinematicState()
        obj = bb(some_str='lol', some_scalar=7.5, some_subobj=bb(x=4, y=5, z=10))
        op  = CreateComplexObject(Path('my_obj'), obj)
        self.assertIn('some_str', op.mod_paths)
        self.assertIn('some_scalar', op.mod_paths)
        self.assertIn('some_subobj', op.mod_paths)
        self.assertIn('some_subobj/x', op.mod_paths)
        self.assertIn('some_subobj/y', op.mod_paths)
        self.assertIn('some_subobj/z', op.mod_paths)
        self.assertEquals(op.mod_paths['some_str'],     ('my_obj','some_str'))
        self.assertEquals(op.mod_paths['some_scalar'],  ('my_obj','some_scalar'))
        self.assertEquals(op.mod_paths['some_subobj'],  ('my_obj','some_subobj'))
        self.assertEquals(op.mod_paths['some_subobj/x'],('my_obj','some_subobj','x'))
        self.assertEquals(op.mod_paths['some_subobj/y'],('my_obj','some_subobj','y'))
        self.assertEquals(op.mod_paths['some_subobj/z'],('my_obj','some_subobj','z'))
        ks.apply_operation(op, 'create my_obj')
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
        ks  = KinematicState()
        ks.apply_operation(CreateSingleValue('my_var', 5), 'create my_var')
        ks.apply_operation(CreateSingleValue('vec_a', spw.vector3(1,0,0)),  'create vec_a')
        ks.apply_operation(CreateSingleValue('vec_b', spw.vector3(0,1,0)),  'create vec_b')

        with self.assertRaises(Exception):
            op = CallFunctionOperator('sin_of_my_var', sin, lol=Path('my_var'))

        op = CallFunctionOperator('sin_of_my_var', sin, expr=Path('my_var'))
        self.assertIn('expr', op.args_paths)
        self.assertEquals(op.args_paths['expr'], 'my_var')
        ks.apply_operation(op, 'compute sin of my_var')
        self.assertTrue(ks.has_data('sin_of_my_var'))
        self.assertEquals(ks.get_data('sin_of_my_var'), sin(5))
        ks.remove_operation('compute sin of my_var')
        self.assertFalse(ks.has_data('sin_of_my_var'))

        with self.assertRaises(Exception):
            op = CallFunctionOperator('cross_of_a_b', cross, u=Path('vec_a'), c=Path('lol'))

        op = CallFunctionOperator('cross_of_a_b', cross, u=Path('vec_a'), v=Path('vec_b'))
        self.assertIn('u', op.args_paths)
        self.assertEquals(op.args_paths['u'], 'vec_a')
        self.assertIn('v', op.args_paths)
        self.assertEquals(op.args_paths['v'], 'vec_b')
        ks.apply_operation(op, 'compute cross of vectors')
        self.assertTrue(ks.has_data('cross_of_a_b'))
        self.assertEquals(ks.get_data('cross_of_a_b'), spw.vector3(0,0,1))
        ks.remove_operation('compute cross of vectors')
        self.assertFalse(ks.has_data('cross_of_a_b'))

if __name__ == '__main__':
    ut.main()