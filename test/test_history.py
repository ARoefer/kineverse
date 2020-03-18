import unittest as ut

from kineverse.model.history import *
from kineverse.utils import bb


def fake_op(deps, mods):
    return bb(args_paths={n: Path(p) for n, p in enumerate(deps)},
               mod_paths={n: Path(p) for n, p in enumerate(mods)})

op_c_a  = fake_op([], ['a'])
op_c_ae = fake_op([], ['a', 'e'])
op_c_bc = fake_op(['a'], ['c', 'b'])
op1     = fake_op(['a', 'b', 'c'], ['e', 'f'])
op2     = fake_op(['a', 'e'], ['e'])
op3     = fake_op(['e', 'b'], ['g'])
op4     = fake_op(['e'], ['g', 'b'])
op5     = fake_op(['a'], ['d', 'f'])
op6     = fake_op(['a'], ['d', 'f', 'e'])
op7     = fake_op(['a'], ['f', 'e'])
op8     = fake_op(['a', 'x'], ['d', 'f', 'e'])


class TestTimeline(ut.TestCase):
    def test_sorting(self):
        l = [100, 34, -77, -5, 0, 3.14, 42]
        t = Timeline(l)
        for x, y in zip(t, sorted(l)):
            self.assertEqual(x, y)

    def test_get_floor(self):
        l = [1, 2, 3, 4, 33, 84, 91, 100]
        t = Timeline(l)

        self.assertEqual(t.get_floor(2)[1],     2)
        self.assertEqual(t.get_floor(4)[1],     4)
        self.assertEqual(t.get_floor(83)[1],   33)
        self.assertEqual(t.get_floor(200)[1], 100)
        self.assertEqual(t.get_floor(0)[1],  None)

    def test_get_ceil(self):
        l = [1, 2, 3, 4, 33, 84, 91, 100]
        t = Timeline(l)

        self.assertEqual(t.get_ceil(2)[1],      2)
        self.assertEqual(t.get_ceil(4)[1],      4)
        self.assertEqual(t.get_ceil(34)[1],    84)
        self.assertEqual(t.get_ceil(200)[1], None)
        self.assertEqual(t.get_ceil(0)[1],      1)

    def test_get_ceil_odd_error_case(self):
        l = [1, 2, 3]
        t = Timeline(l)

        self.assertEqual(t.get_ceil(2)[0],      1)


class TestHistory(ut.TestCase):
    def test_get_time_stamp(self):
        c1 = Chunk(1, op_c_a)
        c2 = Chunk(2, op_c_bc)
        c3 = Chunk(3, op1)
        h  = History([c1, c2, c3])
        self.assertEqual(h.get_time_stamp(), 4)
        self.assertEqual(h.get_time_stamp(before=1),    0)
        self.assertEqual(h.get_time_stamp(before=c1),   0)
        self.assertEqual(h.get_time_stamp(before=2),  1.5)
        self.assertEqual(h.get_time_stamp(before=c2), 1.5)
        self.assertEqual(h.get_time_stamp(after=1),   1.5)
        self.assertEqual(h.get_time_stamp(after=c1),  1.5)
        self.assertEqual(h.get_time_stamp(after=3),     4)
        self.assertEqual(h.get_time_stamp(after=c3),    4)

    def test_insert(self):
        c1 = Chunk(1,   op_c_a)
        c2 = Chunk(2,   op_c_bc)
        c3 = Chunk(3,   op1)
        c4 = Chunk(4,   op3)
        c5 = Chunk(3.5, op4)
        h  = History([c1, c2])

        with self.assertRaises(Exception):
            h.insert_chunk(c4)

        h.insert_chunk(c3)
        self.assertIn(c3, c1.dependents)
        self.assertIn(c3, c2.dependents)

        h.insert_chunk(c4)
        self.assertIn(c4, c2.dependents)
        self.assertIn(c4, c3.dependents)

        h.insert_chunk(c5)
        self.assertNotIn(c4, c2.dependents)
        self.assertIn(c4, c3.dependents)
        self.assertIn(c4, c5.dependents)

    def test_remove(self):
        c1 = Chunk(1,   op_c_a)
        c2 = Chunk(2,   op_c_bc)
        c3 = Chunk(3,   op1)
        c4 = Chunk(4,   op3)
        c5 = Chunk(3.5, op4)
        h  = History([c1, c2, c3])

        with self.assertRaises(Exception):
            h.remove_chunk(c2)

        h.insert_chunk(c5)
        h.insert_chunk(c4)

        self.assertNotIn(c4, c2.dependents)

        h.remove_chunk(c5)
        self.assertIn(c4, c2.dependents)

    def test_replace(self):
        c1 = Chunk(1,   op_c_ae)
        cB = Chunk(2,   op5)
        cG = Chunk(2,   op6)
        cF = Chunk(2,   op7)
        cF2= Chunk(2,   op8)
        c5 = Chunk(3,   op4)
        h  = History([c1, cB, c5])        

        with self.assertRaises(Exception):
            h.replace_chunk(cB, Chunk(4, op5))

        with self.assertRaises(Exception):
            h.replace_chunk(cB, cF)

        with self.assertRaises(Exception):
            h.replace_chunk(cB, cF2)

        h.replace_chunk(cB, cG)
        self.assertIn(c5, cG.dependents)


if __name__ == '__main__':
    ut.main()
