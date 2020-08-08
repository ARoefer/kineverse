import unittest as ut

from kineverse.utils import bb
from kineverse.operations.basic_operations import *
from kineverse.gradients.gradient_math     import *
from kineverse.model.articulation_model    import ArticulationModel


class TestOperations(ut.TestCase):
    def test_memo_correctness(self):
        def add(a, b):
            return a + b

        def mul(a, b):
            return a * b

        km = ArticulationModel()
        p  = Path('my_var')
        op_a = CreateValue(p, 5)
        op_b = ExecFunction(p, mul, p, 2)
        op_c = ExecFunction(p, add, p, 3)

        km.apply_operation('create my_var', op_a)
        km.apply_operation('my_var*=2', op_b)
        km.clean_structure()

        self.assertEquals(km.get_data(p), 10)

        km.apply_operation_before('my_var+=3', 'my_var*=2', op_c)
        km.clean_structure()

        self.assertEquals(km.get_data(p), 16)

    def test_add_single(self):
        km = ArticulationModel()
        p  = Path('my_var')
        op = CreateValue(p, 5)
        km.apply_operation('create my_var', op)
        self.assertTrue(km.has_data(p))
        self.assertEquals(km.get_data(p), 5)
        km.remove_operation('create my_var')
        self.assertFalse(km.has_data(p))


    def test_add_object(self):
        km  = ArticulationModel()
        obj = bb(some_str='lol', some_scalar=7.5, some_subobj=bb(x=4, y=5, z=10))
        op  = CreateValue(Path('my_obj'), obj)
        op.execute(km, 0)
        self.assertIn('output', op.output_paths)
        self.assertIn(Path('some_scalar'), op.output_paths['output'])
        self.assertIn(Path('some_subobj'), op.output_paths['output'])
        self.assertIn(Path('some_subobj/x'), op.output_paths['output'])
        self.assertIn(Path('some_subobj/y'), op.output_paths['output'])
        self.assertIn(Path('some_subobj/z'), op.output_paths['output'])
        self.assertIn(Path('my_obj/some_str'),     op.full_mod_paths)
        self.assertIn(Path('my_obj/some_scalar'),  op.full_mod_paths)
        self.assertIn(Path('my_obj/some_subobj'),  op.full_mod_paths)
        self.assertIn(Path('my_obj/some_subobj/x'),op.full_mod_paths)
        self.assertIn(Path('my_obj/some_subobj/y'),op.full_mod_paths)
        self.assertIn(Path('my_obj/some_subobj/z'),op.full_mod_paths)
        km.apply_operation('create my_obj', op)
        self.assertTrue(km.has_data('my_obj'))
        self.assertEquals(km.get_data('my_obj/some_str'), 'lol')
        self.assertEquals(km.get_data('my_obj/some_scalar'), 7.5)
        self.assertEquals(km.get_data('my_obj/some_subobj/x'), 4)
        self.assertEquals(km.get_data('my_obj/some_subobj/y'), 5)
        self.assertEquals(km.get_data('my_obj/some_subobj/z'), 10)
        km.remove_operation('create my_obj')
        self.assertFalse(km.has_data('my_obj'))
        self.assertFalse(km.has_data('my_obj/some_scalar'))
        self.assertFalse(km.has_data('my_obj/some_subobj'))
        self.assertFalse(km.has_data('my_obj/some_subobj/x'))
        self.assertFalse(km.has_data('my_obj/some_subobj/y'))
        self.assertFalse(km.has_data('my_obj/some_subobj/z'))

    def test_add_fn_call(self):
        km  = ArticulationModel()
        km.apply_operation('create my_var', CreateValue('my_var', 5))
        km.apply_operation('create vec_a',  CreateValue('vec_a', vector3(1,0,0)))
        km.apply_operation('create vec_b',  CreateValue('vec_b', vector3(0,1,0)))

        with self.assertRaises(Exception):
            op = ExecFunction(sin)

        op = ExecFunction('sin_of_my_var', sin, Path('my_var'))
        self.assertIn(Path('my_var'), op.dependencies)
        km.apply_operation('compute sin of my_var', op)
        self.assertTrue(km.has_data('sin_of_my_var'))
        self.assertEquals(km.get_data('sin_of_my_var'), sin(5))
        km.remove_operation('compute sin of my_var')
        self.assertFalse(km.has_data('sin_of_my_var'))

        with self.assertRaises(Exception):
            op = ExecFunction('lol', cross, Path('vec_a'))

        op = ExecFunction('cross_of_a_b', cross, Path('vec_a'), Path('vec_b'))
        self.assertIn('u', op._exec_args)
        self.assertEquals(op._exec_args['u'], Path('vec_a'))
        self.assertIn('v', op._exec_args)
        self.assertEquals(op._exec_args['v'], Path('vec_b'))
        km.apply_operation('compute cross of vectors', op)
        self.assertTrue(km.has_data('cross_of_a_b'))
        self.assertTrue(eq_expr(km.get_data('cross_of_a_b'), vector3(0,0,1)))
        km.remove_operation('compute cross of vectors')
        self.assertFalse(km.has_data('cross_of_a_b'))

if __name__ == '__main__':
    ut.main()
