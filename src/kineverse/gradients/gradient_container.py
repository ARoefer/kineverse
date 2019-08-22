import giskardpy.symengine_wrappers as spw

from kineverse.gradients.diff_logic import get_diff_symbol, get_int_symbol
from kineverse.json_serializable    import JSONSerializable


class GradientContainer(JSONSerializable):
    def __init__(self, expr, gradient_exprs=None):
        if type(expr) == GradientContainer:
            self.expr      = expr.expr
            self.gradients = expr.gradients.copy()
            if gradient_exprs is not None:
                self.gradients.update(gradient_exprs) 
        else:
            self.expr      = expr if type(expr) is not GradientContainer else expr.expr
            self.gradients = gradient_exprs if gradient_exprs is not None else {}
        self.free_symbols = expr.free_symbols if hasattr(expr, 'free_symbols') else set()
        self.free_diff_symbols = {get_diff_symbol(s) for s in self.free_symbols if get_diff_symbol(s) not in self.gradients}

    def do_full_diff(self):
        for fs in self.free_diff_symbols.copy():
            self[fs]

    def _json_data(self, json_dict):
        json_dict.update({'expr': self.expr, 
                          'gradient_exprs': {str(k): d for k, d in self.gradients.items()}})

    @classmethod
    def json_factory(cls, expr, gradient_exprs):
        return cls(expr, {spw.Symbol(k.encode('utf-8')): v for k, v in gradient_exprs.items()})

    def __copy__(self):
        return GradientContainer(self.expr, {k: g for k, g in self.gradients.items()})

    def subs(self, subs):
        return GradientContainer(self.expr.subs(subs), 
                                {s: g.subs(subs) for s, g in self.gradients.items() 
                                                 if get_int_symbol(s) not in subs})

    def __contains__(self, symbol):
        return symbol in self.gradients or symbol in self.free_diff_symbols

    def __getitem__(self, symbol):
        if symbol in self.gradients:
            return self.gradients[symbol]
        elif symbol in self.free_diff_symbols:
            new_term = self.expr.diff(get_int_symbol(symbol))
            self[symbol] = new_term
            return new_term
        else:
            raise Exception('Cannot reproduce or generate gradient terms for variable "{}".\n  Free symbols: {}\n  Free diff symbols: {}'.format(symbol, self.free_symbols, self.free_diff_symbols))

    def __setitem__(self, symbol, expr):
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
        return GradientContainer(self.expr * other, {s: d * other for s, d in self.gradients.items()})

    def __iadd__(self, other):
        if type(other) == GradientContainer:
            self.expr += other.expr
            for k, v in other.gradients.items():
                if k in self.gradients:
                    self.gradients[k] += v
                else:
                    self.gradients[k]  = v
        else:
            self.expr += other
            if hasattr(other, 'free_symbols'):
                for f in other.free_symbols:
                    if get_diff_symbol(f) in self.gradients:
                        self.gradients[get_diff_symbol(f)] += self.expr.diff(f)
        return self

    def __isub__(self, other):
        if type(other) == GradientContainer:
            self.expr += other.expr
            for k, v in other.gradients.items():
                if k in self.gradients:
                    self.gradients[k] -= v
                else:
                    self.gradients[k]  = v
        else:
            self.expr -= other
            if hasattr(other, 'free_symbols'):
                for f in other.free_symbols:
                    if get_diff_symbol(f) in self.gradients:
                        self.gradients[get_diff_symbol(f)] -= self.expr.diff(f)
        return self

    def __imul__(self, other):
        if type(other) == GradientContainer:
            self.expr *= other.expr
            for k, v in other.gradients.items():
                if k in self.gradients:
                    self.gradients[k] += v * self.expr
                else:
                    self.gradients[k]  = v * self.expr
        else:
            temp           = self * other
            self.expr      = temp.expr
            self.gradients = temp.gradients
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
                    gradients[s] += d * self.expr * spw.log(self.expr) * (self.expr ** (other.expr - 1))
                else:
                    gradients[s] = spw.log(self.expr) * d * (self.expr ** other.expr)
            return GradientContainer(self.expr**other.expr, gradients)
        return GradientContainer(self.expr**other, {s: d * other * (self.expr** (other - 1)) for s, d in self.gradients.items()})


    def __str__(self):
        return '{} ({})'.format(str(self.expr), ', '.join([str(k) for k in self.gradients.keys()]))

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