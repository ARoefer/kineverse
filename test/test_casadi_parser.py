import unittest as ut

import kineverse.gradients.gradient_math as gm

import kineverse.casadi_parser as cp


class TestCasadiParser(ut.TestCase):

    def test_consume_neutral(self):
        self.assertEquals('', cp.consume_neutral(' \t\n   '))
        self.assertEquals('lol', cp.consume_neutral('   lol'))
        self.assertEquals('lol\n', cp.consume_neutral('   lol\n'))

    def test_consume_name(self):
        self.assertEquals((',none', 'foo4'), cp.consume_name('foo4,none'))
        self.assertEquals((' none', 'foo4'), cp.consume_name('foo4 none'))
        self.assertEquals((' none', 'https://foo4_lol'), cp.consume_name('https://foo4_lol none'))

        with self.assertRaises(Exception) as context:
            cp.consume_name('4foo')

        with self.assertRaises(Exception) as context:
            cp.consume_name(' lol')

        with self.assertRaises(Exception) as context:
            cp.consume_name('https://')

        with self.assertRaises(Exception) as context:
            cp.consume_name('https:// ')

    def test_consume_number(self):
        self.assertEquals(('', 145), cp.consume_number('145'))
        self.assertEquals(('', -145), cp.consume_number('-145'))
        self.assertEquals(('', 145.54), cp.consume_number('145.54'))
        self.assertEquals(('', 145e+20), cp.consume_number('145e+20'))
        self.assertEquals(('', -145e-21), cp.consume_number('-145e-21'))
        self.assertEquals(('lol', 145), cp.consume_number('145lol'))
        self.assertEquals((' lol', 145), cp.consume_number('145 lol'))

    def test_parse_local(self):
        self.assertEquals((',foo', '12'), cp.consume_local('12,foo'))
        self.assertEquals(('-12,foo', ''), cp.consume_local('-12,foo'))

    def test_parse_atom(self):
        self.assertEquals((',lol', True), cp.parse_atom('@5,lol', {'5': True}))

    def test_single_value(self):
        x, y, z = [gm.Symbol(a) for a in 'xyz']

        a = x +  4 * y
        b = x * -4
        c = x / 2 * y
        d = x - 2 * y

        self.assertEquals(repr(a), repr(cp.parse_casadi(repr(a))))
        self.assertEquals(repr(b), repr(cp.parse_casadi(repr(b))))
        self.assertEquals(repr(c), repr(cp.parse_casadi(repr(c))))
        self.assertEquals(repr(d), repr(cp.parse_casadi(repr(d))))

    def test_vector_value(self):
        x, y, z = [gm.Symbol(a) for a in 'xyz']

        a = gm.dot(gm.frame3_axis_angle(gm.vector3(1, 0, 0), x, gm.point3(y, 0, 0)), gm.point3(4, -2, z))

        self.assertEquals(repr(a), repr(cp.parse_casadi(repr(a))))

    def test_vector_value(self):
        x, y, z = [gm.Symbol(a) for a in 'xyz']

        a = gm.dot(gm.frame3_axis_angle(gm.vector3(1, 0, 0), x, gm.point3(y, 0, 0)),
                   gm.translation3(4, -2, z), gm.rotation3_rpy(x, 0, z))

        self.assertEquals(repr(a), repr(cp.parse_casadi(repr(a))))

if __name__ == '__main__':
    ut.main()
