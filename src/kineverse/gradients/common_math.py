
if SYM_MATH_ENGINE == 'CASADI':
    import casadi as ca

    def Symbol(data):
        if isinstance(data, str) or isinstance(data, unicode):
            return ca.SX.sym(data)
        return ca.SX(data)


    def Matrix(data):
    try:
        return ca.SX(data)
    except NotImplementedError:
        if hasattr(data, u'shape'):
            m = ca.SX(*data.shape)
        else:
            x = len(data)
            if isinstance(data[0], list) or isinstance(data[0], tuple):
                y = len(data[0])
            else:
                y = 1
            m = ca.SX(x, y)
        for i in range(m.shape[0]):
            if y > 1:
                for j in range(m.shape[1]):
                    try:
                        m[i, j] = data[i][j]
                    except:
                        m[i, j] = data[i, j]
            else:
                m[i] = data[i]
        return m

    zeros = ca.SX.zeros 

    def free_symbols(expression):
        return ca.symvar(expression)

    def subs(expr, subs_dict):
        if hasattr(expr, 'subs') and callable(expr.subs):
            return expr.subs(subs_dict)
        return expr

elif SYM_MATH_ENGINE == 'SYMENGINE':
    import symengine as se

    Symbol = se.Symbol
    Matrix = se.Matrix

    zeros  = se.zeros

    def free_symbols(expression):
        return expression.free_symbols

    def subs(expr, subs_dict):
        if hasattr(expr, 'subs') and callable(expr.subs):
            return expr.subs(subs_dict)
        return expr
