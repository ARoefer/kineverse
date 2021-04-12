import kineverse.gradients.common_math as cm

neutral_chars = {' ', '\n', '\t'}
alph_chars    = set('abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ')
digits        = set('0123456789')
first_name_chars = alph_chars.union({'_'})
name_chars = alph_chars.union(digits).union({'_'})

def sq(expr):
    return expr ** 2

cm.ca.sq = sq

def parse_casadi(instr):
    if instr[:3] != 'SX(':
        raise Exception('Casadi expressions are expected to start with "SX(" got "{}"'.format(instr[:3]))

    locals = {}
    instr = consume_neutral(instr[3:])
    
    while True:
        if instr[0] == '@':
            instr = consume_neutral(parse_local(instr, locals))
            if instr[0] != ',':
                raise Exception('Expected "," after local definition')
            instr = consume_neutral(instr[1:])
        else:
            instr, expr = parse_matrix(instr, locals)
            instr = consume_neutral(instr)
            if instr[0] != ')':
                raise Exception('Expected ")" after expression definition, got "{}"'.format(instr[0]))
            return cm.Matrix(expr)


def parse_local(instr, locals):
    if instr[0] != '@':
        raise Exception('Expected local definition to start with "@" got "{}"'.format(instr[0]))

    instr, number = consume_local(instr[1:])
    if instr[0] != '=':
        raise Exception('Local definition requires "=" after name declaration. Local variable is {}, got character "{}"'.format(number, instr[0]))

    if number in locals:
        raise Exception('Double definition of local variable "{}"'.format(number))

    instr, expr = parse_expr(instr[1:], locals)
    locals[number] = expr
    return instr


def parse_matrix(instr, locals):
    instr = consume_neutral(instr)
    if instr[0] == '[':
        acc = []
        while True:
            instr, inner = parse_matrix(instr[1:], locals)
            acc.append(inner)
            instr = consume_neutral(instr)
            if instr[0] == ',':
                instr = instr[1:]
            elif instr[0] == ']':
                return instr[1:], acc
    elif instr[0] in first_name_chars.union(digits).union({'-', '(', '@'}):
        instr, expr = parse_expr(instr, locals)
        return instr, expr
    else:
        raise Exception('Illegal start character for matrix parsing: "{}"'.format(instr[0]))

def parse_expr(instr, locals):
    instr, acc = parse_factor(instr, locals)
    instr = consume_neutral(instr)
    while instr[0] == '+' or instr[0] == '-':
        if instr[0] == '+':
            instr, f = parse_factor(instr[1:], locals)
            acc += f
        else:
            instr, f = parse_factor(instr[1:], locals)
            acc -= f
        instr = consume_neutral(instr)
    return instr, acc

def parse_factor(instr, locals):
    instr, acc = parse_atom(instr, locals)
    instr = consume_neutral(instr)
    while instr[0] == '*' or instr[0] == '/':
        if instr[0] == '*':
            instr, f = parse_atom(instr[1:], locals)
            acc *= f
        else:
            instr, f = parse_atom(instr[1:], locals)
            acc /= f
        instr = consume_neutral(instr)
    return instr, acc

def parse_argument_list(instr, locals):
    out = []
    while True:
        instr = consume_neutral(instr)
        instr, expr = parse_expr(instr, locals)
        out.append(expr)
        instr = consume_neutral(instr)
        if instr[0] != ',':
            break
        instr = instr[1:]
    return instr, out


def parse_atom(instr, locals):
    instr = consume_neutral(instr)
    if instr[0] == '-':
        instr, expr = parse_atom(instr[1:], locals)
        return instr, -expr
    elif instr[0] in digits:
        return consume_number(instr)
    elif instr[0] in first_name_chars:
        instr, name = consume_name(instr)
        if instr[0] == '(':
            instr, arg_list = parse_argument_list(instr[1:], locals)
            instr = consume_neutral(instr)
            if instr[0] != ')':
                raise Exception('Expected closing ")"')
            return instr[1:], getattr(cm.ca, name)(*arg_list)
        return instr, cm.Symbol(name)
    elif instr[0] == '@':
        instr, local = consume_local(instr[1:])
        if local not in locals:
            raise Exception('Undefined local variable "{}"'.format(local))
        return instr, locals[local]
    elif instr[0] == '(':
        instr, expr = parse_expr(instr[1:], locals)
        instr = consume_neutral(instr)
        if instr[0] != ')':
            raise Exception('Expected closing ")" while parsing expression.')
        return instr[1:], expr
    else:
        raise Exception('Cannot parse atom starting with "{}"'.format(instr[0]))

def consume_local(instr):
    cutoff = 0
    while cutoff < len(instr) and instr[cutoff] in digits:
        cutoff += 1
    return instr[cutoff:], instr[:cutoff]

def consume_number(instr):
    cutoff = 0
    generator = int
    if instr[0] == '-':
        cutoff += 1

    while cutoff < len(instr) and instr[cutoff] in digits:
        cutoff += 1

    if cutoff < len(instr) and instr[cutoff] == '.':
        cutoff += 1
        while cutoff < len(instr) and instr[cutoff] in digits:
            cutoff += 1
        generator = float
    
    if cutoff < len(instr) - 2 and instr[cutoff:cutoff + 2] in {'e-', 'e+'}:
        cutoff += 2
        while cutoff < len(instr) and instr[cutoff] in digits:
            cutoff += 1
        generator = float

    return instr[cutoff:], generator(instr[:cutoff])

def consume_name(instr):
    cutoff = 0
    if instr[cutoff] not in first_name_chars:
        raise Exception('Failed to parse variable name. Illegal first char "{}"'.format(instr[cutoff]))
    is_uri = False
    while cutoff < len(instr):
        if instr[cutoff] in name_chars:
            cutoff += 1
        elif instr[cutoff] == ':' and not is_uri:
            if len(instr) >= cutoff + 4:
                if instr[cutoff:cutoff + 3] == '://':
                    cutoff += 3
                    is_uri = True
                else:
                    raise Exception('Expected URI infix but got: "{}"'.format(instr[cutoff:cutoff + 3]))
            else:
                raise Exception('Detected ":" for URI infix, but remaining string is too short.')
        else:
            break
    if is_uri and instr[:cutoff][-1] == '/':
        raise Exception('URI can not end in "/". Identified name: "{}"'.format(instr[:cutoff]))
    return instr[cutoff:], instr[:cutoff]


def consume_neutral(instr):
    cutoff = 0
    while cutoff < len(instr) and instr[cutoff] in neutral_chars:
        cutoff += 1

    return instr[cutoff:]
