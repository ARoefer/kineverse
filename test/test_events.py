import unittest as ut

import kineverse.gradients.common_math  as cm

from kineverse.model.event_model        import EventModel, Path
from kineverse.model.articulation_model import Constraint
from kineverse.operations.operation     import Operation
from kineverse.utils                    import bb


class EmptyOperation(Operation):
    def __init__(self):
        super(EmptyOperation, self).__init__({})

    def _execute_impl(self):
        self.constraints = {}


class TestEventModel(ut.TestCase):

    def test_modifcation_events(self):
        km = EventModel()

        self.change_a = None
        self.change_b = None
        self.change_c = None
        self.change_c_f = None
        self.change_c_g = None
        def cb_change_a(data):
            self.change_a = data

        def cb_change_b(data):
            self.change_b = data

        def cb_change_c(data):
            self.change_c = data

        def cb_change_c_f(data):
            self.change_c_f = data

        def cb_change_c_g(data):
            self.change_c_g = data

        km.register_on_model_changed(Path('a'), cb_change_a)
        km.register_on_model_changed(Path('b'), cb_change_b)
        km.register_on_model_changed(Path('c'), cb_change_c)
        km.register_on_model_changed(Path('c/f'), cb_change_c_f)
        km.register_on_model_changed(Path('c/g'), cb_change_c_g)

        km.set_data('a', 5)
        km.dispatch_events()
        self.assertEquals(self.change_a, 5)
        self.assertEquals(self.change_b, None)

        km.set_data('b', 6)
        km.dispatch_events()
        self.assertEquals(self.change_b, 6)
        self.assertEquals(self.change_c, None)

        self.change_b = None
        km.set_data('b', 6)
        km.dispatch_events()
        self.assertEquals(self.change_b, None)

        c_1 = bb(f='lol', g=False)
        c_2 = bb(f='kek', g=False)
        c_3 = bb(f='foo', g=True)
        km.set_data('c', c_1)
        km.dispatch_events()
        self.assertEquals(self.change_c, c_1)
        self.assertEquals(self.change_c_f, 'lol')
        self.assertEquals(self.change_c_g, False)

        self.change_c = None
        self.change_c_g = None

        km.set_data('c/f', 'kek')
        km.dispatch_events()
        self.assertEquals(self.change_c, c_1)
        self.assertEquals(self.change_c_f, 'kek')
        self.assertEquals(self.change_c_g, None)

        km.set_data('c', c_3)
        km.dispatch_events()
        self.assertEquals(self.change_c, c_3)
        self.assertEquals(self.change_c_f, 'foo')
        self.assertEquals(self.change_c_g, True)

        km.remove_data('c')
        km.dispatch_events()
        self.assertEquals(self.change_c, None)
        self.assertEquals(self.change_c_f, None)
        self.assertEquals(self.change_c_g, None)

        self.change_b = None
        km.deregister_on_model_changed(cb_change_b)
        km.set_data('b', 42)
        km.dispatch_events()
        self.assertEquals(self.change_b, None)


    def test_constraint_changed(self):
        km = EventModel()

        s_a_constraints = {}
        s_b_constraints = {}
        s_c_constraints = {}
        s_d_constraints = {}
        s_ba_constraints = {}
        s_dcb_constraints = {}

        def gen_cb(*cb_dicts):
            def cb(name, c):
                for cbd in cb_dicts:
                    if c is not None:
                        cbd[name] = c
                    elif name in cbd:
                        del cbd[name]

            return cb

        s_a, s_b, s_c, s_d = [cm.Symbol(x) for x in 'abcd']

        km.register_on_constraints_changed({s_a}, gen_cb(s_a_constraints))
        km.register_on_constraints_changed({s_b}, gen_cb(s_b_constraints))
        km.register_on_constraints_changed({s_c}, gen_cb(s_c_constraints))
        km.register_on_constraints_changed({s_d}, gen_cb(s_d_constraints))
        km.register_on_constraints_changed({s_a, s_b}, gen_cb(s_ba_constraints))
        km.register_on_constraints_changed({s_d, s_c, s_b}, gen_cb(s_dcb_constraints))

        c1 = Constraint(-1, 1, s_a)
        c2 = Constraint(-1, 1, s_a * s_b)
        c3 = Constraint(-1, 1, s_c)
        c4 = Constraint(-1, 1, s_d * s_c * s_b)

        # Insert constraint based on single variable
        km.add_constraint('c1', c1)
        km.dispatch_events()
        self.assertIn('c1', s_a_constraints)
        self.assertIn('c1', s_ba_constraints)
        self.assertEquals(s_a_constraints['c1'],  c1)
        self.assertEquals(s_ba_constraints['c1'], c1)
        self.assertNotIn('c1', s_b_constraints)
        self.assertNotIn('c1', s_c_constraints)
        self.assertNotIn('c1', s_d_constraints)
        self.assertNotIn('c1', s_dcb_constraints)

        # Insert constraint based on different single variable
        km.add_constraint('c3', c3)
        km.dispatch_events()
        self.assertIn('c3', s_c_constraints)
        self.assertIn('c3', s_dcb_constraints)
        self.assertEquals(s_c_constraints['c3'],  c3)
        self.assertEquals(s_dcb_constraints['c3'], c3)
        self.assertNotIn('c3', s_a_constraints)
        self.assertNotIn('c3', s_b_constraints)
        self.assertNotIn('c3', s_d_constraints)
        self.assertNotIn('c3', s_ba_constraints)

        # Insert constraint based on two variables
        km.add_constraint('c2', c2)
        km.dispatch_events()
        self.assertIn('c2', s_a_constraints)
        self.assertIn('c2', s_b_constraints)
        self.assertIn('c2', s_ba_constraints)
        self.assertIn('c2', s_dcb_constraints)
        self.assertEquals(s_a_constraints['c2'],  c2)
        self.assertEquals(s_b_constraints['c2'],  c2)
        self.assertEquals(s_ba_constraints['c2'],  c2)
        self.assertEquals(s_dcb_constraints['c2'], c2)
        self.assertNotIn('c2', s_c_constraints)
        self.assertNotIn('c2', s_d_constraints)

        # Replace constraint, while changing its symbol set
        km.add_constraint('c2', c4)
        km.dispatch_events()
        self.assertNotIn('c2', s_a_constraints)
        self.assertIn('c2', s_c_constraints)
        self.assertIn('c2', s_d_constraints)
        self.assertIn('c2', s_b_constraints)
        self.assertIn('c2', s_dcb_constraints)
        self.assertEquals(s_c_constraints['c2'],  c4)
        self.assertEquals(s_b_constraints['c2'],  c4)
        self.assertEquals(s_d_constraints['c2'],  c4)
        self.assertEquals(s_dcb_constraints['c2'],  c4)

        # Remove constraint
        km.remove_constraint('c2')
        km.dispatch_events()
        self.assertNotIn('c2', s_c_constraints)
        self.assertNotIn('c2', s_d_constraints)
        self.assertNotIn('c2', s_b_constraints)
        self.assertNotIn('c2', s_dcb_constraints)


    def test_operation_changed(self):
        km = EventModel()

        self.op_tracker = {}

        def cb_op(tag, applied):
            self.op_tracker[tag] = applied

        km.register_on_operation_changed('foo', cb_op)
        km.register_on_operation_changed('bar', cb_op)
        km.register_on_operation_changed('kek \w*', cb_op)
        km.register_on_operation_changed('hah \w* something', cb_op)
        km.register_on_operation_changed('hah lol \w*', cb_op)

        km.apply_operation('nothing', EmptyOperation())
        km.dispatch_events()
        self.assertNotIn('nothing', self.op_tracker)

        km.apply_operation('foo', EmptyOperation())
        km.dispatch_events()
        self.assertIn('foo', self.op_tracker)
        self.assertTrue(self.op_tracker['foo'])
        
        km.remove_operation('foo')
        km.dispatch_events()
        self.assertIn('foo', self.op_tracker)
        self.assertFalse(self.op_tracker['foo'])

        km.apply_operation('kek', EmptyOperation())
        km.dispatch_events()
        self.assertNotIn('kek', self.op_tracker)

        km.apply_operation('kek wonderful', EmptyOperation())
        km.dispatch_events()
        self.assertIn('kek wonderful', self.op_tracker)
        self.assertTrue(self.op_tracker['kek wonderful'])

        km.apply_operation_before('hah abc something', 'kek', EmptyOperation())
        km.dispatch_events()
        self.assertIn('hah abc something', self.op_tracker)
        self.assertTrue(self.op_tracker['hah abc something'])        

        km.apply_operation_after('hah lol nothing', 'kek wonderful', EmptyOperation())
        km.dispatch_events()
        self.assertIn('hah lol nothing', self.op_tracker)
        self.assertTrue(self.op_tracker['hah lol nothing'])


if __name__ == '__main__':
    ut.main()
