"""
The gradient_container module implements the GradientContainer, a realization of
Kineverse's augmented gradient concept. It also implements GradientMatrix to handle
matrices storing augmented gradients.
"""

import kineverse.gradients.common_math as cm

from kineverse.gradients.diff_logic import DiffSymbol, IntSymbol, Symbol, get_symbol_type, TYPE_UNKNOWN
from kineverse.json_serializable    import JSONSerializable


class DerivativeException(Exception):
    pass

class GradientContainer(JSONSerializable):
    """Conserves additional derivative information in addition to the one that can be derived from the main experssion.
    """
    def __init__(self, expr, gradient_exprs=None):
        """Constructor. Stores given expression an optional gradient information.

        :param           expr: Core expression to be stored.
        :type            expr: int, float, GradientContainer, symengine type
        :param gradient_exprs: Dictionary of custom derivatives. Mapping: symengine.Symbol -> Expression
        :type  gradient_exprs: dict
        """
        if type(expr) == GradientContainer:
            self.expr      = expr.expr
            self.gradients = expr.gradients.copy()
            if gradient_exprs is not None:
                self.gradients.update(gradient_exprs) 
        else:
            self.expr      = expr if type(expr) is not GradientContainer else expr.expr
            self.gradients = gradient_exprs if gradient_exprs is not None else {}
        self.free_symbols = cm.free_symbols(expr)
        self.free_diff_symbols = {DiffSymbol(s) for s in self.free_symbols if get_symbol_type(s) != TYPE_UNKNOWN and DiffSymbol(s) not in self.gradients}

    @property
    def diff_symbols(self):
        """Returns the set of symbols for which a gradient can be provided."""
        return self.free_diff_symbols.union(set(self.gradients.keys()))

    def diff(self, x):
        """Provides the differentiation of the main term w.r.t. the given Symbol.
        :param x: Symbol w.r.t.w. to differentiate.
        :type  x: Symbol.
        :return: Derivative expression, or 0
        """
        xdot = DiffSymbol(x)
        if xdot in self.diff_symbols:
            return self[xdot]
        return 0

    def do_full_diff(self):
        """Computes all derivatives for the expression."""
        for fs in self.free_diff_symbols.copy():
            self[fs]

    def _json_data(self, json_dict):
        json_dict.update({'expr': self.expr, 
                          'gradient_exprs': {str(k): d for k, d in self.gradients.items()}})

    @classmethod
    def json_factory(cls, expr, gradient_exprs):
        """Instantiates a GradientContainer from a JSON data structure."""
        return cls(expr, {Symbol(k.encode('utf-8')): v for k, v in gradient_exprs.items()})

    def __copy__(self):
        return GradientContainer(self.expr, {k: g for k, g in self.gradients.items()})

    def subs(self, subs):
        """Substitutes variables for other expressions.
        NOTE: THIS DOES NOT WORK WITH CASADI, unless the replacements are floats.

        :param subs: Dictionary of substitutions. Mapping: Symbol -> Expression
        :type  subs: dict
        :return: Gradient resulting from substitution
        :rtype: GradientContainer
        """
        return GradientContainer(cm.subs(self.expr, subs), 
                                {s: g.subs(subs) for s, g in self.gradients.items() 
                                                 if IntSymbol(s) not in subs})

    def __contains__(self, symbol):
        """Checks whether a derivative expression can be derived for the given symbol.

        :param symbol: Symbol to compute derivative for
        :type  symbol: Symbol
        :rtype: bool
        """
        return symbol in self.gradients or symbol in self.free_diff_symbols

    def __getitem__(self, symbol):
        """Computes the derivative for a given symbol.

        :param symbol: Symbol to compute derivative for
        :type  symbol: Symbol
        :return: Derivative expression for symbol
        :rtype:  Expression
        :raises: DerivativeException
        """
        if symbol in self.gradients:
            return self.gradients[symbol]
        elif symbol in self.free_diff_symbols:
            new_term = cm.diff(self.expr, IntSymbol(symbol))
            self[symbol] = new_term
            return new_term
        else:
            # return 0
            raise Exception('Cannot reproduce or generate gradient terms for variable "{}".\n  Free symbols: {}\n  Free diff symbols: {}'.format(symbol, self.free_symbols, self.free_diff_symbols))

    def __setitem__(self, symbol, expr):
        """Adds custom derivative for given symbol.

        :param symbol: Symbol to add derivative for
        :type  symbol: Symbol
        :param   expr: Derivative expression for symbol
        :type    expr: Expression
        """
        if symbol in self.free_diff_symbols:
            self.free_diff_symbols.remove(symbol)
        self.gradients[symbol] = expr


    def __neg__(self):
        return GradientContainer(-self.expr, {s: -d for s, d in self.gradients.items()})

    def __radd__(self, other):
        return self.__add__(other)

    def __add__(self, other):
        if type(other) == GradientContainer:
            gradients = self.gradients.copy()
            for s, d in other.gradients.items():
                if s in gradients:
                    gradients[s] += d
                else:
                    gradients[s] = d
            return GradientContainer(self.expr + other.expr, gradients)        
        return GradientContainer(self.expr + other, self.gradients.copy())

    def __rsub__(self, other):
        return GradientContainer(other) - self

    def __sub__(self, other):
        if type(other) == GradientContainer:
            gradients = self.gradients.copy()
            for s, d in other.gradients.items():
                if s in gradients:
                    gradients[s] -= d
                else:
                    gradients[s] = -d
            return GradientContainer(self.expr - other.expr, gradients)
        return GradientContainer(self.expr - other, {s: -d for s, d in self.gradients.items()})

    def __rmul__(self, other):
        return self.__mul__(other)

    def __mul__(self, other):
        if type(other) == GradientContainer:
            gradients = {s: d * other.expr for s, d in self.gradients.items()}
            for s, d in other.gradients.items():
                if s in gradients:
                    gradients[s] += d * self.expr
                else:
                    gradients[s] = d * self.expr
            return GradientContainer(self.expr * other.expr, gradients)
        elif type(other) in cm.matrix_types and cm.is_matrix(other):
            return GradientMatrix(other) * self
        return GradientContainer(self.expr * other, {s: d * other for s, d in self.gradients.items()})

    def __iadd__(self, other):
        if type(other) == GradientContainer:
            self.expr += other.expr
            for k, v in other.gradients.items():
                if k in self.gradients:
                    self.gradients[k] += v
                else:
                    self.gradients[k]  = v
            self.free_diff_symbols |= other.free_diff_symbols
        else:
            self.expr += other
            fs = cm.free_symbols(other)
            for f in fs:
                if DiffSymbol(f) in self.gradients:
                    self.gradients[DiffSymbol(f)] += cm.diff(self.expr, f)
                else:
                    self.free_diff_symbols.add(DiffSymbol(f))
        self.free_symbols = cm.free_symbols(self.expr)
        return self

    def __isub__(self, other):
        if type(other) == GradientContainer:
            self.expr += other.expr
            for k, v in other.gradients.items():
                if k in self.gradients:
                    self.gradients[k] -= v
                else:
                    self.gradients[k]  = v
            self.free_diff_symbols |= other.free_diff_symbols
        else:
            self.expr -= other
            fs = cm.free_symbols(other)
            for f in fs:
                if DiffSymbol(f) in self.gradients:
                    self.gradients[DiffSymbol(f)] -= cm.diff(self.expr, f)
                else:
                    self.free_diff_symbols.add(DiffSymbol(f))
        self.free_symbols = cm.free_symbols(self.expr)
        return self

    def __imul__(self, other):
        if type(other) == GradientContainer:
            self.expr *= other.expr
            for k, v in other.gradients.items():
                if k in self.gradients:
                    self.gradients[k] += v * self.expr
                else:
                    self.gradients[k]  = v * self.expr
            self.free_diff_symbols |= other.free_diff_symbols
        else:
            temp           = self * other
            self.expr      = temp.expr
            self.gradients = temp.gradients
            self.free_diff_symbols = temp.free_diff_symbols
        self.free_symbols = cm.free_symbols(self.expr)
        return self

    def __div__(self, other):
        return self.__truediv__(other)

    def __truediv__(self, other):
        if type(other) == GradientContainer:
            gradients = {s: d * other.expr for s, d in self.gradients.items()}
            for s, d in other.gradients.items():
                if s in gradients:
                    gradients[s] -= d * self.expr
                else:
                    gradients[s] = -d * self.expr
            return GradientContainer(self.expr / other.expr, {s: d / (other.expr**2) for s, d in gradients.items()})
        return GradientContainer(self.expr / other, {s: d / (other ** 2) for s, d in self.gradients.items()})


    def __pow__(self, other):
        if type(other) == GradientContainer:
            gradients = {s: d * other.expr * (self.expr ** (other.expr - 1)) for s, d in self.gradients.items()}
            for s, d in other.gradients.items():
                if s in gradients:
                    gradients[s] += d * self.expr * cm.log(self.expr) * (self.expr ** (other.expr - 1))
                else:
                    gradients[s] = cm.log(self.expr) * d * (self.expr ** other.expr)
            return GradientContainer(self.expr**other.expr, gradients)
        return GradientContainer(self.expr**other, {s: d * other * (self.expr** (other - 1)) for s, d in self.gradients.items()})

    def __rpow__(self, other):
        if type(other) != GradientContainer:
            return GradientContainer(other) ** self
        raise Exception('Should not have gotten here')


    def __str__(self):
        return '\\/ {} [{}]'.format(str(self.expr), ', '.join([str(k) for k in self.diff_symbols]))

    def __repr__(self):
        return 'G({})'.format(self.expr)

    def __eq__(self, other):
        if type(other) == GradientContainer:
            return self.expr == other.expr

    def __lt__(self, other):
        if type(other) is GradientContainer:
            return self.expr < other.expr
        return self.expr < other

    def __gt__(self, other):
        if type(other) is GradientContainer:
            return self.expr > other.expr
        return self.expr > other

    def __le__(self, other):
        if type(other) is GradientContainer:
            return self.expr <= other.expr
        return self.expr <= other

    def __ge__(self, other):
        if type(other) is GradientContainer:
            return self.expr >= other.expr
        return self.expr >= other

GC = GradientContainer


def copy_nested_list(l):
    """Helper for creating copies of nested lists."""
    return [copy_nested_list(r) if type(r) is list else r.__copy__() for r in l]

def gradient_from_list(l):
    """Creates a (nested) list of GradientContainer from the input."""
    if len(l) == 0:
        return [], []
    elif type(l[0]) == list:
        return [[x.expr if type(x) == GC else x for x in r] for r in l], [[x if type(x) == GC else GC(x) for x in r] for r in l]
    else:
        return [x.expr if type(x) == GC else x for x in l], [x if type(x) == GC else GC(x) for x in l]

def check_correct_matrix(l):
    """Checks the equal dimensionality of nested lists."""
    if len(l) > 0:
        inner_lists = [type(x) is list for x in l]
        if min(inner_lists) is True:
            inner_len = [len(x) for x in l]
            return min(inner_len) == max(inner_len) and min([check_correct_matrix(x) for x in l])
        return max(inner_lists) is False
    return True

def collect_free_symbols(l):
    """Helper function sacking all 'free_symbols' sets from objects in a list, if they exist."""
    out = set()
    for x in l:
        if type(x) == list:
            out |= collect_free_symbols(x)
        else:
            out |= cm.free_symbols(x)
    return out

def collect_diff_symbols(l):
    out = set()
    for x in l:
        if type(x) == list:
            out |= collect_diff_symbols(x)
        else:
            out |= x.diff_symbols
    return out

def floatify_nested_list(l):
    """Extracts the expression member from GradientContainers in a nested list."""
    return [x.expr if type(x) == GC else floatify_nested_list(x) for x in l]


class GradientMatrix(JSONSerializable):
    def __init__(self, expr):
        """Constructor. Takes a matrix in the form of a symengine.Matrix, or nested list and stores it as matrix of GradientContainer.

        :type expr: symengine.Matrix, list
        """
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
        elif type(expr) in cm.matrix_types:
            self.expr      = [[GC(x) for x in r] for r in cm.to_list(expr)]
            self._nrows    = expr.shape[0]
            self._ncols    = expr.shape[1]
        
        self.free_symbols = collect_free_symbols(self.expr)
        #self.free_diff_symbols = {DiffSymbol(s) for s in self.free_symbols if DiffSymbol(s) not in self.gradients}

    @property
    def diff_symbols(self):
        """Returns the set of symbols for which a gradient can be provided."""
        return collect_diff_symbols(self.expr)

    def _json_data(self, json_dict):
        json_dict.update({'expr': self.expr})

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
        return cm.ncolsself.expr, () * cm.nrowsself.expr, ()

    def __iter__(self):
        return iter(sum(self.expr, []))

    def __str__(self):
        return '\n'.join([str(r) for r in self.expr])

    def nrows(self):
        """Returns the number of rows of the stored matrix.
        :rtype: int
        """
        return self._nrows

    def ncols(self):
        """Returns the number of columns of the stored matrix.
        :rtype: int
        """
        return self._ncols

    @property
    def shape(self):
        return (self._nrows, self._ncols)

    @property
    def T(self):
        """Transpose of the matrix."""
        return GradientMatrix([[self.expr[y][x].__copy__() 
                                for y in range(self._nrows)] 
                                for x in range(self._ncols)])

    def __neg__(self):
        return GradientMatrix([[-e for e in row] for row in self.expr])

    def __radd__(self, other):
        return self.__add__(other)

    def __add__(self, other):
        if type(other) == GradientMatrix:
            return GradientMatrix([[a + b for a, b in zip(self.expr[x], other.expr[x])] for x in range(len(self.expr))])
        elif type(other) in cm.matrix_types:
            return self + GradientMatrix(other)
        else:
            return GradientMatrix([[x + other for x in r] for r in self.expr])

    def __rsub__(self, other):
        if type(other) in cm.matrix_types:
            return GradientMatrix(other) - self


    def __sub__(self, other):
        if type(other) == GradientMatrix:
            return GradientMatrix([[a - b for a, b in zip(self.expr[x], other.expr[x])] for x in range(len(self.expr))])
        elif type(other) in cm.matrix_types:
            return self - GradientMatrix(other)
        else:
            return GradientMatrix([[x - other for x in r] for r in self.expr])

    # Component-wise multiplication
    def __mul__(self, other):
        if cm.is_matrix(other) or type(other) == GradientMatrix:
            if self.shape == other.shape:
                return GradientMatrix([[self[y, x] * other[y, x] for x in range(self.shape[1])] 
                                                                 for y in range(self.shape[0])])
            raise Exception('Matrices for component-wise multiplication need to be of equal shape. '
                            'Shapes are: {}, {}'.format(self.shape, other.shape))
        else:
            return GradientMatrix([[g * other for g in r] for r in self.expr])

    # Matrix multiplication
    def dot(self, other):
        if type(other) == GradientMatrix:
            expr = [[GC(0) for y in range(other._ncols)] for x in range(self._nrows)]
            for x in range(other._ncols):
                for z in range(self._nrows):
                    for y in range(other._nrows):
                        expr[z][x] += self.expr[z][y] * other.expr[y][x]
            return GradientMatrix(expr)
        else:
            return self.dot(GradientMatrix(other))

    # Rule introducing a preceding conversion step, similar to rmul etc
    def rdot(self, other):
        if type(other) != GradientMatrix:
            return GradientMatrix(other).dot(self)
        return other.dot(self) # This should actually never be the case


    def __rmul__(self, other):
        if type(other) in cm.matrix_types:
            return GradientMatrix(other) * self
        return self.__mul__(other)

    def __div__(self, other):
        return GradientMatrix([[g / other for g in r] for r in self.expr])

    def __repr__(self):
        return '\n'.join([repr(r) for r in self.expr])

    def __eq__(self, other):
        if type(other) == GradientMatrix and self._nrows == other._nrows and self._ncols == other._ncols:
            return min([min([x == y for x, y in zip(self.expr[x], other.expr[x])]) for x in range(self._nrows)])
        return False

    def __copy__(self):
        return GradientMatrix(copy_nested_list(self.expr))

    def __deepcopy__(self, memo):
        return self.__copy__()

    def to_sym_matrix(self):
        """Converts the stored matrix to a symengine.Matrix, erasing all additional gradient information.
        :rtype: symengine.Matrix
        """
        return cm.Matrix(floatify_nested_list(self.expr))
