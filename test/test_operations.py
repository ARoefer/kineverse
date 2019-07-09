import unittest as ut

from kineverse.operations.basic_operations import *
from kineverse.gradients.gradient_math     import *
from kineverse.operations.kinematic_state  import KinematicState


class TestOperations(ut.TestCase):
    def test_add_single(self):
        ks = KinematicState()
        op = CreateSingleValue('my_var', 5)
        ks.apply_operation(op, 'create my_var')
        self.assertTrue(ks.has_data('my_var'))
        self.assertEquals(ks.get_data('my_var'), 5)
        ks.remove_operation('create my_var')
        self.assertFalse(ks.has_data('my_var'))

    def test_add_object(self):
        pass

    def test_add_fn_call(self):
        pass


if __name__ == '__main__':
    ut.main()