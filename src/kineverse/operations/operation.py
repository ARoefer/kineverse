
# Every action needs to declare the fields it depends on and the ones it modifies
class Operation(object):
    def __init__(self, name, args, modifications, **kwargs):
        self.args_paths = {a : kwargs[a] for a in args}
        self.mod_paths  = {m : kwargs[m] for m in modifications}
        self._memento   = {} # Memento of the modified expressions

    def apply(self, ks):
        self._memento = {m: ks.get_data(p) for m, p in self.mod_paths.items() if ks.has_data(p)}
        self._apply(ks, **{k: e for k, e in self.mod_paths.items() + self.args_paths.items()})

    def _apply(self, ks, **kwargs):
        raise NotImplementedError

    def revoke(self, ks):
        for p, e in self._memento.values():
            ks.set_data(p, e)
        for p in {p for k, p in self.mod_paths.items() if k not in self._memento}:
            ks.remove_data(p)