import giskardpy.symengine_wrappers as spw

Symbol = spw.Symbol

TYPE_UNKNOWN  = 0
TYPE_POSITION = 1
TYPE_VELOCITY = 2
TYPE_ACCEL    = 3
TYPE_JERK     = 4
TYPE_SNAP     = 5
TYPE_SUFFIXES = {'_p': TYPE_POSITION, 
                 '_v': TYPE_VELOCITY, 
                 '_a': TYPE_ACCEL,
                 '_j': TYPE_JERK,
                 '_s': TYPE_SNAP}
TYPE_SUFFIXES_INV = {v: k for k, v in TYPE_SUFFIXES.items()}


class CastException(Exception):
    pass    

class ConversionException(Exception):
    pass

def create_symbol(symbol, stype):
    """Adds proper type suffix to the given symbol.

    :param symbol: Symbol to be typed
    :type  symbol: str, symengine.Symbol
    :param  stype: Enum type to assign to symbol
    :type   stype: int
    :return: Typed version of symbol
    :rtype: symengine.Smybol
    """
    if type(symbol) is not str and type(symbol) is not Symbol:
        symbol = symbol.to_symbol()

    if stype not in TYPE_SUFFIXES_INV:
        raise Exception('Can not create symbol for type {}: Type id not defined.'.format(stype))
    return Symbol('{}{}'.format(str(symbol), TYPE_SUFFIXES_INV[stype]))

def create_pos(symbol):
    """Shorthand for creating a position symbol.

    :param symbol: Symbol to be typed
    :type  symbol: str, symengine.Symbol
    :return: Symbol typed as position
    :rtype: symengine.Symbol
    """
    return create_symbol(symbol, TYPE_POSITION)

def create_vel(symbol):
    """Shorthand for creating a velocity symbol.

    :param symbol: Symbol to be typed
    :type  symbol: str, symengine.Symbol
    :return: Symbol typed as velocity
    :rtype: symengine.Symbol
    """
    return create_symbol(symbol, TYPE_VELOCITY)

def create_acc(symbol):
    """Shorthand for creating an acceleration symbol.

    :param symbol: Symbol to be typed
    :type  symbol: str, symengine.Symbol
    :return: Symbol typed as acceleration
    :rtype: symengine.Symbol
    """
    return create_symbol(symbol, TYPE_ACCEL)

def create_jerk(symbol):
    """Shorthand for creating a jerk symbol.

    :param symbol: Symbol to be typed
    :type  symbol: str, symengine.Symbol
    :return: Symbol typed as jerk
    :rtype: symengine.Symbol
    """
    return create_symbol(symbol, TYPE_JERK)

def create_snap(symbol):
    """Shorthand for creating a snap symbol.

    :param symbol: Symbol to be typed
    :type  symbol: str, symengine.Symbol
    :return: Symbol typed as snap
    :rtype: symengine.Symbol
    """
    return create_symbol(symbol, TYPE_SNAP)

def erase_type(symbol):
    """Removes type suffix from symbol if one is present.

    :param symbol: Symbol to untype
    :type  symbol: str, symengine.Symbol
    :return: Typeless symbol
    :rtype: symengine.Symbol
    """
    st = get_symbol_type(symbol)
    if st != TYPE_UNKNOWN:
        return Symbol(str(symbol)[:-len(TYPE_SUFFIXES_INV[st])])
    return symbol

def get_symbol_type(symbol):
    """Returns the enum value of a symbol's type.

    :param symbol: Symbol to inspect
    :type  symbol: str, symengine.Symbol
    :return: Enum value of symbol's type
    :rtype: int
    """
    return TYPE_SUFFIXES[str(symbol)[-2:]] if str(symbol)[-2:] in TYPE_SUFFIXES else TYPE_UNKNOWN

def get_diff_symbol(symbol):
    """Returns the derivative symbol of the given symbol.

    :param symbol: Symbol to convert
    :type  symbol: str, symengine.Symbol
    :return: Derivative of given symbol
    :rtype: symengine.Symbol
    :raises: CastException if symbol is of unknown type or a snap
    """
    s_type = get_symbol_type(symbol)
    if s_type == TYPE_UNKNOWN or s_type == TYPE_SNAP:
        raise CastException('Cannot generate derivative symbol for {}! The type is {}'.format(symbol, s_type))
    return Symbol('{}{}'.format(str(symbol)[:-2], TYPE_SUFFIXES_INV[s_type + 1]))

def get_int_symbol(symbol):
    """Returns the integral symbol of the given symbol.

    :param symbol: Symbol to convert
    :type  symbol: str, symengine.Symbol
    :return: Inetgral of given symbol
    :rtype: symengine.Symbol
    :raises: CastException if symbol is of unknown type or a position
    """
    s_type = get_symbol_type(symbol)
    if s_type == TYPE_UNKNOWN or s_type == TYPE_POSITION:
        raise CastException('Cannot generate integrated symbol for {}! The type is {}'.format(symbol, s_type))
    return Symbol('{}{}'.format(str(symbol)[:-2], TYPE_SUFFIXES_INV[s_type - 1]))

def get_conversion(a_space1, a_space2, dt=1):
    if erase_type(a_space1) != erase_type(a_space2):
        raise ConversionException('Cannot determine conversion factor from "{}" to "{}" because they do not refer to the same symbol.'.format(a_space1, a_space2))

    t1 = get_symbol_type(a_space1)
    t2 = get_symbol_type(a_space2)

    if t1 == TYPE_UNKNOWN:
        raise ConversionException('Type of can not be determined {}'.format(a_space1))

    if t2 == TYPE_UNKNOWN:
        raise ConversionException('Type of can not be determined {}'.format(a_space2))

    return dt ** (t2 - t1)


Position     = create_pos
Velocity     = create_vel
Acceleration = create_acc
Jerk         = create_jerk
Snap         = create_snap