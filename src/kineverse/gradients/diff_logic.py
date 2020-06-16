"""
The diff_logic module implements Kineverse's typed symbol system.
"""

import kineverse.gradients.common_math as cm

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

Symbol = cm.Symbol

class CastException(Exception):
    pass    

def create_symbol(symbol, stype):
    """Adds proper type suffix to the given symbol.

    :param symbol: Symbol to be typed
    :type  symbol: str, Symbol
    :param  stype: Enum type to assign to symbol
    :type   stype: int
    :return: Typed version of symbol
    :rtype: symengine.Smybol
    """
    if type(symbol) is not str and not cm.is_symbol(symbol):
        symbol = symbol.to_symbol()

    if stype not in TYPE_SUFFIXES_INV:
        raise Exception('Can not create symbol for type {}: Type id not defined.'.format(stype))
    return cm.Symbol('{}{}'.format(str(symbol), TYPE_SUFFIXES_INV[stype]))

def Position(symbol):
    """Shorthand for creating a position symbol.

    :param symbol: Symbol to be typed
    :type  symbol: str, Symbol
    :return: Symbol typed as position
    :rtype: Symbol
    """
    return create_symbol(symbol, TYPE_POSITION)

def Velocity(symbol):
    """Shorthand for creating a velocity symbol.

    :param symbol: Symbol to be typed
    :type  symbol: str, Symbol
    :return: Symbol typed as velocity
    :rtype: Symbol
    """
    return create_symbol(symbol, TYPE_VELOCITY)

def Acceleration(symbol):
    """Shorthand for creating an acceleration symbol.

    :param symbol: Symbol to be typed
    :type  symbol: str, Symbol
    :return: Symbol typed as acceleration
    :rtype: Symbol
    """
    return create_symbol(symbol, TYPE_ACCEL)

def Jerk(symbol):
    """Shorthand for creating a jerk symbol.

    :param symbol: Symbol to be typed
    :type  symbol: str, Symbol
    :return: Symbol typed as jerk
    :rtype: Symbol
    """
    return create_symbol(symbol, TYPE_JERK)

def Snap(symbol):
    """Shorthand for creating a snap symbol.

    :param symbol: Symbol to be typed
    :type  symbol: str, Symbol
    :return: Symbol typed as snap
    :rtype: Symbol
    """
    return create_symbol(symbol, TYPE_SNAP)

def erase_type(symbol):
    """Removes type suffix from symbol if one is present.

    :param symbol: Symbol to untype
    :type  symbol: str, Symbol
    :return: Typeless symbol
    :rtype: Symbol
    """
    st = get_symbol_type(symbol)
    if st != TYPE_UNKNOWN:
        return cm.Symbol(str(symbol)[:-len(TYPE_SUFFIXES_INV[st])])
    return symbol

def get_symbol_type(symbol):
    """Returns the enum value of a symbol's type.

    :param symbol: Symbol to inspect
    :type  symbol: str, Symbol
    :return: Enum value of symbol's type
    :rtype: int
    """
    return TYPE_SUFFIXES[str(symbol)[-2:]] if str(symbol)[-2:] in TYPE_SUFFIXES else TYPE_UNKNOWN

def DiffSymbol(symbol):
    """Returns the derivative symbol of the given symbol.

    :param symbol: Symbol to convert
    :type  symbol: str, Symbol
    :return: Derivative of given symbol
    :rtype: Symbol
    :raises: CastException if symbol is of unknown type or a snap
    """
    s_type = get_symbol_type(symbol)
    if s_type == TYPE_UNKNOWN or s_type == TYPE_SNAP:
        raise CastException('Cannot generate derivative symbol for {}! The type is {}'.format(symbol, s_type))
    return cm.Symbol('{}{}'.format(str(symbol)[:-2], TYPE_SUFFIXES_INV[s_type + 1]))

def IntSymbol(symbol):
    """Returns the integral symbol of the given symbol.

    :param symbol: Symbol to convert
    :type  symbol: str, Symbol
    :return: Inetgral of given symbol
    :rtype: Symbol
    :raises: CastException if symbol is of unknown type or a position
    """
    s_type = get_symbol_type(symbol)
    if s_type == TYPE_UNKNOWN or s_type == TYPE_POSITION:
        raise CastException('Cannot generate integrated symbol for {}! The type is {}'.format(symbol, s_type))
    return cm.Symbol('{}{}'.format(str(symbol)[:-2], TYPE_SUFFIXES_INV[s_type - 1]))
