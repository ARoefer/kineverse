import giskardpy.symengine_wrappers as spw
from kineverse.gradients.diff_logic         import get_diff_symbol
from kineverse.gradients.gradient_container import GradientContainer as GC

symengine_matrix_types = set([spw.sp.DenseMatrix, 
                              spw.sp.ImmutableMatrix, 
                              spw.sp.ImmutableDenseMatrix, 
                              spw.sp.MatrixBase, 
                              spw.sp.MutableDenseMatrix])

def is_scalar(expr):
    return type(expr) == int or type(expr) == float or expr.is_Add or expr.is_AlgebraicNumber or expr.is_Atom or expr.is_Derivative or expr.is_Float or expr.is_Function or expr.is_Integer or expr.is_Mul or expr.is_Number or expr.is_Pow or expr.is_Rational or expr.is_Symbol or expr.is_finite or expr.is_integer or expr.is_number or expr.is_symbol



def gradient_from_list(l):
    if len(l) == 0:
        return [], []
    elif type(l[0]) == list:
        return [[x.expr if type(x) == GC else x for x in r] for r in l], [[x if type(x) == GC else GC(x) for x in r] for r in l]
    else:
        return [x.expr if type(x) == GC else x for x in l], [x if type(x) == GC else GC(x) for x in l]

def check_correct_matrix(l):
    if len(l) > 0:
        inner_lists = [type(x) is list for x in l]
        if min(inner_lists) is True:
            inner_len = [len(x) for x in l]
            return min(inner_len) == max(inner_len) and min([check_correct_matrix(x) for x in l])
        return max(inner_lists) is False
    return True

def collect_free_symbols(l):
    out = set()
    for x in l:
        if hasattr(x, 'free_symbols'):
            out |= x.free_symbols
        elif type(x) == list:
            out |= collect_free_symbols(x)
    return out

class GradientMatrix(object):
    def __init__(self, expr):
        if type(expr) == list:
            if not check_correct_matrix(expr):
                raise Exception('List passed for Matrix construction is not correctly formed! List: {}'.format(expr))
            
            if len(expr) > 0 and type(expr[0]) != list:
                self.expr  = [[x] if type(x) == GC else [GC(x)] for x in expr]
                self._ncols = 1
                self._nrows = len(expr) 
            else:
                self.expr   = [[x if type(x) == GC else GC(x) for x in r] for r in expr]
                self._nrows = len(expr)
                self._ncols = len(expr[0]) if self._nrows > 0 else 0
        elif type(expr) in symengine_matrix_types:
            self.expr      = [[GC(x) for x in r] for r in expr.tolist()]
            self._nrows    = expr.nrows()
            self._ncols    = expr.ncols()
        
        self.free_symbols = collect_free_symbols(self.expr)
        #self.free_diff_symbols = {get_diff_symbol(s) for s in self.free_symbols if get_diff_symbol(s) not in self.gradients}

    def __getitem__(self, idx):
        if type(idx) == int:
            return self.expr[idx / self._ncols][idx % self._ncols]
        elif type(idx) == slice:
            return sum(self.expr, [])[idx]
        elif type(idx) == tuple:
            if type(idx[0]) == int and type(idx[1]) == int:
                return self.expr[idx[0]][idx[1]]
            return GradientMatrix([r[idx[1]] for r in self.expr[idx[0]]])

    def __setitem__(self, idx, expr):
        if type(idx) == int:
            self.expr[idx / self._ncols][idx % self._ncols] = expr if type(expr) == GC else GC(expr)
        elif type(idx) == tuple:
            self.expr[idx[0]][idx[1]] = expr if type(expr) == GC else GC(expr)

    def __len__(self):
        return self.expr.ncols() * self.expr.nrows()

    def __iter__(self):
        return iter(sum(self.expr, []))

    def __str__(self):
        return '\n'.join([str(r) for r in self.expr])

    def nrows(self):
        return self._nrows

    def ncols(self):
        return self._ncols

    @property
    def T(self):
        return GradientMatrix([[self.expr[y][x].copy() 
                                for y in range(self._nrows)] 
                                for x in range(self._ncols)])

    def __radd__(self, other):
        return self.__add__(other)

    def __add__(self, other):
        if type(other) == GradientMatrix:
            return GradientMatrix([[a + b for a, b in zip(self.expr[x], other.expr[x])] for x in range(len(self.expr))])
        elif type(other) in symengine_matrix_types:
            return self + GradientMatrix(other)
        else:
            return GradientMatrix([[x + other for x in r] for r in self.expr])

    def __rsub__(self, other):
        if type(other) in symengine_matrix_types:
            return GradientMatrix(other) - self


    def __sub__(self, other):
        if type(other) == GradientMatrix:
            return GradientMatrix([[a - b for a, b in zip(self.expr[x], other.expr[x])] for x in range(len(self.expr))])
        elif type(other) in symengine_matrix_types:
            return self - GradientMatrix(other)
        else:
            return GradientMatrix([[x - other for x in r] for r in self.expr])

    def __mul__(self, other):
        if type(other) == GradientMatrix:
            expr = [[GC(0) for y in range(other._ncols)] for x in range(self._nrows)]
            for x in range(other._ncols):
                for z in range(self._nrows):
                    for y in range(other._nrows):
                        expr[z][x] += self.expr[z][y] * other.expr[y][x]
            return GradientMatrix(expr)
        elif type(other) in symengine_matrix_types:
            return self * GradientMatrix(other)
        return GradientMatrix([[g * other for g in r] for r in self.expr])
            #raise Exception('Operation {} * {} is undefined.'.format(type(self), type(other)))

    def __rmul__(self, other):
        if type(other) in symengine_matrix_types:
            return GradientMatrix(other) * self
        return self.__mul__(other)

    def __div__(self, other):
        return GradientMatrix([[g / other for g in r] for r in self.gradients])

    def __repr__(self):
        return '\n'.join([repr(r) for r in self.expr])

    def __eq__(self, other):
        if type(other) == GradientMatrix and self._nrows == other._nrows and self._ncols == other._ncols:
            return min([min([x == y for x, y in zip(self.expr[x], other.expr[x])]) for x in range(self._nrows)])
        return False