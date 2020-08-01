from kineverse.gradients.diff_logic import *
from kineverse.type_sets import *



def texify(expr, level=0):
    t = type(expr)
    if t == sp.Symbol:
        st = get_symbol_type(expr)
        if st == TYPE_UNKNOWN:
            return str(expr)
        typeless = erase_type(expr)
        if st == TYPE_POSITION:
            return str(typeless)
        elif st

    elif t == sp.Abs:
        return '\\norm\{{}\}'.format(texify(expr.get_arg(), 0))
    elif t == sp.Add:
        return ' + '.join()
