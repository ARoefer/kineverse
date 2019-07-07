class Operation(object):
    def __init__(self, name, t_exprs, t_args, **paths):
        self.t_exprs = t_exprs
        self.t_args  = t_args
        self.o_exprs = {}
        self.e_paths    = {k: (t, paths[k]) for k, t in self.t_exprs.items()}
        self.args_paths = {k: (t, paths[k]) for k, t in self.t_args.items()}

    def apply(self, ks):
        self.o_exprs = {k: (p, ks.get_typed_expr(p, t)) for k, (t, p) in self.e_paths.items()}
        args = {k: ks.get_typed_expr(p, t) for k, (t, p) in self.args_paths.items()}
        args.update({k: e for k, (_, e) in self.o_exprs.items()})
        self._apply(**args)

    def _apply(self, **kwargs):
        raise NotImplementedError

    def revoke(self, ks):
        for p, e in self.o_exprs.values():
            ks.set_expr(p, e)