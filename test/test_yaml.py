import unittest as ut
from kineverse.yaml_wrapper            import yaml
from kineverse.gradients.gradient_math import *

class TestSymengineYAML(ut.TestCase):

    def test_matrix(self):
        M = spw.Matrix([[1,2,3], ['a', 'b', 'c']])

        M_2 = yaml.full_load(yaml.dump(M))

        for x in range(len(M)):
            self.assertEqual(M[x], M_2[x], 'Matrix elements at index {} are not equal. Elements:\n  M  : {} is {} \n  M_2: {} is {}\n'.format(x, M[x], type(M[x]), M_2[x], type(M_2[x])))


    def test_expression(self):
        e = (sin('a') * 67) / tan(45)

        e_2 = yaml.full_load(yaml.dump(e))
        self.assertEqual(e, e_2)


if __name__ == '__main__':
    ut.main()