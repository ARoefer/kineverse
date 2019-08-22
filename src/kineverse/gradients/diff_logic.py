import giskardpy.symengine_wrappers as spw

TYPE_UNKNOWN  = 0
TYPE_POSITION = 1
TYPE_VELOCITY = 2
TYPE_ACCEL    = 3
TYPE_EFFORT   = 4
TYPE_SUFFIXES = {'_p': TYPE_POSITION, 
                 '_v': TYPE_VELOCITY, 
                 '_a': TYPE_ACCEL,
                 '_e': TYPE_EFFORT}
TYPE_SUFFIXES_INV = {v: k for k, v in TYPE_SUFFIXES.items()}

def create_symbol(symbol, stype):
    if stype not in TYPE_SUFFIXES_INV:
        raise Exception('Can not create symbol for type {}: Type id not defined.'.format(stype))
    return spw.Symbol('{}{}'.format(str(symbol), TYPE_SUFFIXES_INV[stype]))

def create_pos(symbol):
    return create_symbol(symbol, TYPE_POSITION)

def create_vel(symbol):
    return create_symbol(symbol, TYPE_VELOCITY)

def create_acc(symbol):
    return create_symbol(symbol, TYPE_ACCEL)

def create_eff(symbol):
    return create_symbol(symbol, TYPE_EFFORT)

def erase_type(symbol):
    st = get_symbol_type(symbol)
    if st != TYPE_UNKNOWN:
        return spw.Symbol(str(symbol)[:-len(TYPE_SUFFIXES_INV[st])])
    return symbol

def get_symbol_type(symbol):
    return TYPE_SUFFIXES[str(symbol)[-2:]] if str(symbol)[-2:] in TYPE_SUFFIXES else TYPE_UNKNOWN

def get_diff_symbol(symbol):
    s_type = get_symbol_type(symbol)
    if s_type == TYPE_UNKNOWN or s_type == TYPE_EFFORT:
        raise Exception('Cannot generate derivative symbol for {}! The type is {}'.format(symbol, s_type))
    return spw.Symbol('{}{}'.format(str(symbol)[:-2], TYPE_SUFFIXES_INV[s_type + 1]))

def get_int_symbol(symbol):
    s_type = get_symbol_type(symbol)
    if s_type == TYPE_UNKNOWN or s_type == TYPE_POSITION:
        raise Exception('Cannot generate integrated symbol for {}! The type is {}'.format(symbol, s_type))
    return spw.Symbol('{}{}'.format(str(symbol)[:-2], TYPE_SUFFIXES_INV[s_type - 1]))