try:
    f = profile
except NameError:
    def profile(f):
        return f
    import __builtin__ as builtins
    builtins.__dict__['profile'] = profile