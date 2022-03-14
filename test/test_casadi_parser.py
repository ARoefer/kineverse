import unittest as ut

import kineverse.gradients.gradient_math  as gm
import kineverse.casadi_parser            as cp

class TestEventModel(ut.TestCase):

    def test_consume_neutral(self):
        self.assertEqual(cp.consume_neutral(' \n\ta', 0), 3)
        self.assertEqual(cp.consume_neutral(' \n\t  ', 0), 5)

    def test_parse_number(self):
        self.assertEqual(cp.parse_number('0', 0)[0], 0)
        self.assertEqual(cp.parse_number('24', 0)[0], 24)
        self.assertEqual(cp.parse_number('24.75', 0)[0], 24.75)
        self.assertEqual(cp.parse_number('24e3', 0)[0], 24e3)
        self.assertEqual(cp.parse_number('24e-128', 0)[0], 24e-128)
        self.assertEqual(cp.parse_number('24.75e-128', 0)[0], 24.75e-128)

    def test_parse_name(self):
        self.assertEqual(cp.parse_name('lol', 0)[0], 'lol')

    def test_parse_factor(self):
        defs = {'@12': 4}
        self.assertEqual(cp.parse_factor(' -43', 0, defs)[0], -43)
        self.assertEqual(cp.parse_factor('43', 0, defs)[0], 43)
        self.assertEqual(cp.parse_factor('-43.32e4', 0, defs)[0], -43.32e4)
        self.assertEqual(str(cp.parse_factor('  lol ', 0, defs)[0]), 'lol')
        self.assertEqual(cp.parse_factor('  @12 ', 0, defs)[0], 4)
        self.assertEqual(cp.parse_factor(' -@12 ', 0, defs)[0], -4)
        self.assertEqual(str(cp.parse_factor('sin(a)', 0, defs)[0]), str(gm.sin(gm.Symbol('a'))))

    def test_parse_term(self):
        defs = {'@12': 4}
        self.assertEqual(cp.parse_term('@12*3/5-4', 0, defs)[0], defs['@12'] * 3 / 5)
    
    def test_parse_expr(self):
        defs = {'@12': 4}
        self.assertEqual(cp.parse_expr('33e-2+@12*3/5-4', 0, defs)[0], 33e-2 + defs['@12'] * 3 / 5 - 4)

    def test_parse_list(self):
        defs = {'@12': 4}
        self.assertEqual(str(cp.parse_list('[ a, 423e3, @12,66]', 0, defs, cp.parse_expr)[0]), 
                         str([gm.Symbol('a'), 423e3, defs['@12'], 66]))
        
    def test_parse_matrix(self):
        defs = {'@12': 4}
        self.assertEqual(cp.parse_matrix('32', 0, defs)[0], 32)
        self.assertEqual(str(cp.parse_matrix('[32]', 0, defs)[0]), 
                         str(gm.Matrix([32])))

        M = gm.Matrix([[          1, 2, 3],
                       [defs['@12'], 5, 6],
                       [          7, 8, 9]])
        self.assertEqual(str(cp.parse_matrix('[[1,2,3], [@12, 5, 6], [7, 8, 9]]', 0, defs)[0]),
                         str(M))

    def test_parse(self):
        defs = {'@1': 4, '@2': gm.sin(gm.Symbol('a'))}
        ref_1 = gm.cm.ca.SX(defs['@1'])
        M = gm.Matrix([[    1, 2, ref_1],
                       [ref_1, 5, defs['@2']],
                       [    7, 8, defs['@2']]])
        self.assertEqual(str(cp.parse('@1=4, @2=sin(a), ([[1,2,@1], [@1, 5, @2], [7, 8, @2]])')),
                         str(M))

if __name__ == '__main__':
    ut.main()
