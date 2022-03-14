import string

import kineverse.gradients.gradient_math as gm

letters       = set(string.ascii_letters)
digits        = set(string.digits)
name_glyphs   = set(string.ascii_letters + string.digits + '_')
number_glyphs = set(string.digits + '.')

def op_add(a, b):
    return a + b

def op_sub(a, b):
    return a - b

def op_mul(a, b):
    return a * b

def op_div(a, b):
    return a / b

operators = {'+': op_add, '-': op_sub, '*': op_mul, '/': op_div}
functions = {f.__qualname__: f for f in [getattr(gm, n) for n in dir(gm) if callable(getattr(gm, n))]}
functions['sq'] = lambda x: x**2
functions['fabs'] = gm.abs

def parse(string):
    # if string[:3] != 'SX(' or string[-1] == ')':
    #     raise Exception('Casadi expressions need to start with "SX(" and end with ")". '
    #                     f'The given expression starts with "{string[:3]}" and ends with "{string[-1]}"')
    defs = {}

    i = 0
    while True:
        i = consume_neutral(string, i)
        if string[i] != '@':
            break
        n, d, i = parse_def(string, i, defs)
        defs[n] = d
        i = consume_neutral(string, i)
        if string[i] != ',':
            raise Exception(f'Expected "," after parsing of definition got "{string[i]}"')
        i += 1

    return parse_expr(string, i, defs)[0]

def parse_def(s, i, defs):
    if s[i] != '@':
        raise Exception(f'Definitions need to start with "@" but got "{s[i]}"')
    
    int_s, i = parse_int_str(s, i+1)
    
    if s[i] != '=':
        raise Exception(f'Definition expected "=" but got "{s[i]}" during parsing of "@{int_s}"')
    
    i += 1
    e, i = parse_expr(s, i, defs)
    return f'@{int_s}', e, i

def parse_matrix(s, i, defs):
    m, i = parse_nested_list(s, i, defs)
    return gm.Matrix(m), i

def parse_nested_list(s, i, defs):
    i = consume_neutral(s, i)
    if s[i] == '[':
        return parse_list(s, i, defs, inner_op=parse_nested_list)
    else:
        e, i = parse_expr(s, i, defs)
        return e, i

def parse_expr(s, i, defs):
    terms = []
    ops   = []
    while True:
        t, i = parse_term(s, i, defs)
        terms.append(t)
        i = consume_neutral(s, i)
        if i == len(s) or s[i] not in '+-':
            break
        
        ops.append(operators[s[i]])
        i += 1
    
    acc = terms[0]
    for o, t in zip(ops, terms[1:]):
        acc = o(acc, t)
    return acc, i

def parse_list(s, i, defs, inner_op=parse_expr):
    if s[i] not in '([':
        raise Exception(f'Lists need to start with "(" or "[" but got "{s[i]}"')
    sb = s[i]

    l = []
    while True:
        e, i = inner_op(s, i + 1, defs)
        l.append(e)
        i = consume_neutral(s, i)
        if s[i] != ',':
            break
    
    if (sb == '(' and s[i] != ')') or (sb == '[' and s[i] != ']'):
        raise Exception(f'List needs to close with ")" or "]", but got "{s[i]}"')
    return l, i + 1

def parse_term(s, i, defs):
    factors = []
    ops     = []
    while True:
        f, i = parse_factor(s, i, defs)
        factors.append(f)
        i = consume_neutral(s, i)
        if i == len(s) or s[i] not in '*/':
            break
        
        ops.append(operators[s[i]])
        i += 1
    
    acc = factors[0]
    for o, f in zip(ops, factors[1:]):
        acc = o(acc, f)
    return acc, i

def parse_factor(s, i, defs):
    i = consume_neutral(s, i)
    sign, i = parse_sign(s, i)

    if s[i] == '(':
        e, i = parse_expr(s, i + 1, defs)
        i = consume_neutral(s, i)
        if s[i] != ')':
            raise Exception(f'Expected ")" but got "{s[i]}" at index {i}')
        return sign * e, i + 1
    elif s[i] == '[':
        m, i = parse_matrix(s, i, defs)
        return sign * m, i
    elif s[i] == '@':
        int_s, i = parse_int_str(s, i + 1)
        return sign * defs[f'@{int_s}'], i
    elif s[i] in letters:
        name, i = parse_name(s, i)
        if i < len(s) and s[i] == '(':
            l, i = parse_list(s, i, defs)
            if not name in functions:
                raise Exception(f'Unknown function "{name}"')
            return sign * functions[name](*l), i
        return sign * gm.Symbol(name), i
    else:
        number, i = parse_number(s, i)
        return sign * number, i

def parse_sign(s, i):
    return (-1, i+1) if s[i] == '-' else (1, i)

def parse_number(s, i):
    int_s, i = parse_int_str(s, i)
    if i == len(s) or (s[i] not in '.e'):
        return int(int_s), i
    
    decimal_s = '0'
    if s[i] == '.':
        decimal_s, i = parse_int_str(s, i + 1)
    
    if len(s) == i or s[i] != 'e':
        return float(f'{int_s}.{decimal_s}'), i
    
    e_sign, i = parse_sign(s, i + 1)
    exp_s, i  = parse_int_str(s, i)

    return float(f'{int_s}.{decimal_s}e{e_sign * int(exp_s)}'), i

def parse_int_str(s, i):
    x = i
    while i < len(s) and s[i] in digits:
        i += 1
    return s[x:i], i

def parse_name(s, i):
    x = i
    while i < len(s) and s[i] in name_glyphs:
        i += 1
    return s[x:i], i

def consume_neutral(s, i):
    while i < len(s) and s[i] in {' ', '\t', '\n'}:
        i += 1

    return i