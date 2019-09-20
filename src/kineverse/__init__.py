try:
    f = profile
except NameError:
    def profile(f):
        return f
    import __builtin__ as builtins
    builtins.__dict__['profile'] = profile

TYPE_CHECKING = True

def type_check(*types):
    def dec_check(f):
        if TYPE_CHECKING:
            def checked_f(*args):
                args_start = 0 if f.func_code.co_varnames[0] != 'self' else 1
                if min([isinstance(v, t) for t, v in zip(types[:len(args) - args_start], args[args_start:])]) is False:
                    raise Exception('Function {} called with wrong types:\n  Signature: {}\n      Given: {}'.format(f.func_name, ', '.join([str(t) for t in types[:len(args) - args_start]]), ', '.join([str(type(a)) for a in args[args_start:]])))
                return f(*args)
            return checked_f
        else:
            return f
    return dec_check

builtins.__dict__['type_check'] = type_check