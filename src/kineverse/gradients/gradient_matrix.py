import giskardpy.symengine_wrappers as spw
from kineverse.gradients.gradient_container import GradientContainer as GC

def is_scalar(expr):
    return type(expr) == int or type(expr) == float or expr.is_Add or expr.is_AlgebraicNumber or expr.is_Atom or expr.is_Derivative or expr.is_Float or expr.is_Function or expr.is_Integer or expr.is_Mul or expr.is_Number or expr.is_Pow or expr.is_Rational or expr.is_Symbol or expr.is_finite or expr.is_integer or expr.is_number or expr.is_symbol

def gradient_from_list(l):
    if len(l) == 0:
        return [], []
    elif type(l[0]) == list:
        return [[x.expr if type(x) == GradientContainer else x for x in r] for r in l], [[x if type(x) == GradientContainer else GradientContainer(x) for x in r] for r in l]
    else:
        return [x.expr if type(x) == GradientContainer else x for x in l], [x if type(x) == GradientContainer else GradientContainer(x) for x in l]

class GradientMatrix(object):
    def __init__(self, expr, gradient_exprs=None):
        if type(expr) == list:
            m_list, self.gradients = gradient_from_list(expr)
            self.expr              = spw.sp.Matrix(m_list)
        else:
            self.expr         = expr
            self.gradients    = [[gradient_exprs[y][x] if type(gradient_exprs[y][x]) == GC else GC(self.expr[y, x]) for x in range(self.expr.ncols())] for y in range(self.expr.nrows())] if gradient_exprs is not None else [[GC(e) for e in expr[x,:]] for x in range(expr.nrows())]

        if len(self.gradients) != self.expr.nrows() or len(self.gradients[0]) != self.expr.ncols():
            raise Exception('Gradient dimensions do not match matrix dimensions!\n Matrix: {}, {}\n Gradient: {}, {}'.format(self.expr.nrows(), self.expr.ncols(), len(self.gradients), len(self.gradients[0])))

        self.free_symbols = self.expr.free_symbols
        self.free_diff_symbols = {get_diff_symbol(s) for s in self.free_symbols if get_diff_symbol(s) not in self.gradients}

    def __contains__(self, symbol):
        return symbol in self.gradients or symbol in self.free_diff_symbols

    def __getitem__(self, idx):
        if type(idx) == int:
            return self.gradients[idx / self.expr.ncols()][idx % self.expr.ncols()]
        elif type(idx) == slice:
            return sum(self.gradients, [])[idx]
        elif type(idx) == tuple:
            return GradientMatrix(self.expr[idx], self.gradients[idx[0]][idx[1]])

    def __setitem__(self, idx, expr):
        if type(idx) == int:
            if type(expr) == GC:
                self.expr[idx] = expr.expr
                self.gradients[idx / self.expr.ncols()][idx % self.expr.ncols()] = expr
            else:
                self.expr[idx] = expr
                self.gradients[idx / self.expr.ncols()][idx % self.expr.ncols()] = GC(expr)
        elif type(idx) == tuple:
            if type(expr) == GC:
                self.expr[idx] = expr.expr
                self.gradients[idx[0]][idx[1]] = expr
            else:
                self.expr[idx] = expr
                self.gradients[idx[0]][idx[1]] = GC(expr)

    def __len__(self):
        return self.expr.ncols() * self.expr.nrows()

    def __iter__(self):
        return iter(sum(self.gradients, []))

    def __str__(self):
        return '\n'.join(['[{}]'.format(', '.join([str(x) for x in r])) for r in self.gradients])

    @property
    def T(self):
        return GradientMatrix(self.expr.T, [[self.gradients[y][x].copy() 
                                             for y in range(self.expr.nrows())] 
                                             for x in range(self.expr.ncols())])

    def __radd__(self, other):
        return self.__add__(other)

    def __add__(self, other):
        if type(other) == GradientMatrix:
            return GradientMatrix(self.expr + other.expr, 
                                 [[a + b for a, b in zip(self.gradients[x], other.gradients[x])] for x in range(len(self.gradients))])
        else:
            return self + GradientMatrix(other)

    def __rsub__(self, other):
        return GradientMatrix(other) - self

    def __sub__(self, other):
        if type(other) == GradientMatrix:
            return GradientMatrix(self.expr - other.expr, 
                                 [[a - b for a, b in zip(self.gradients[x], other.gradients[x])] for x in range(len(self.gradients))])
        else:
            return self - GradientMatrix(other)

    def __mul__(self, other):
        if type(other) == GradientMatrix:
            gradients = [[GC(0) for y in range(other.expr.ncols())] for x in range(self.expr.nrows())]
            for x in range(other.expr.ncols()):
                for z in range(self.expr.nrows()):
                    for y in range(other.expr.nrows()):
                        gradients[z][x] += self.gradients[y][x]
            return GradientMatrix(self.expr * other.expr, gradients)
        elif other.is_Matrix:
            return self * GradientMatrix(other)
        elif is_scalar(other):
            return GradientMatrix(self.expr * other, [[g * other for g in r] for r in self.gradients])
        elif type(other) == GC:
            return GradientMatrix(self.expr * other.expr, [[g * other for g in r] for r in self.gradients])
        else:
            raise Exception('Operation {} * {} is undefined.'.format(type(self), type(other)))

    def __div__(self, other):
        if type(other) == GC:
            return GradientMatrix(self.expr / other.expr, [[g / other for g in r] for r in self.gradients])
        elif is_scalar(other):
            return GradientMatrix(self.expr / other, [[g / other for g in r] for r in self.gradients])
        else:
            raise Exception('Operation {} / {} is undefined.'.format(type(self), type(other)))