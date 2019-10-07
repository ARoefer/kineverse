import re

from kineverse.model.kinematic_model import KinematicModel
from kineverse.model.paths           import Path, PathDict, PathSet
    
def _dispatch_model_events(pdict, data, key, call_tracker=set()):
    for cb in pdict.value:
        if cb not in call_tracker:
            cb(data)
            call_tracker.add(cb)
    if len(key) == 0:
        for k, p in pdict.items():
            _dispatch_model_events(p, Path(k).get_data_no_throw(data), key, call_tracker)
    else:
        _dispatch_model_events(pdict.get_sub_dict(key[0]), key[:1].get_data_no_throw(data), key[1:], call_tracker)


class EventModel(KinematicModel):
    def __init__(self):
        super(EventModel, self).__init__()
        self.model_change_callbacks    = PathDict(set(), default_factory=set)
        self.constraint_callbacks      = {}
        # Inverse mapping of callbacks to their triggers, for easier removal 
        self._callback_path_registry   = {}
        self._callback_symbol_registry = {}

        self._constraint_callback_batch = {}
        self._constraint_discard_batch  = {}
        self._callback_batch = PathSet()

        self.tag_callbacks = {}
        self._callback_matcher_registry = {}
        self.__in_dispatch_mode = False
        self.__callback_additions = []
        self.__callback_removals  = []


    def apply_operation(self, tag, op):
        super(EventModel, self).apply_operation(tag, op)
        self._on_operation_changed(tag, True)

    def apply_operation_before(self, tag, before_tag, op):
        super(EventModel, self).apply_operation_before(tag, before_tag, op)
        self._on_operation_changed(tag, True)

    def apply_operation_after(self, tag, after_tag, op):
        super(EventModel, self).apply_operation_after(tag, after_tag, op)
        self._on_operation_changed(tag, True)

    def remove_operation(self, tag):
        super(EventModel, self).remove_operation(tag)
        self._on_operation_changed(tag, False)

    def _on_operation_changed(self, tag, applied):
        call_tracker = set()
        for m, cbs in self.tag_callbacks.items():
            if m.match(tag) is not None:
                for cb in cbs:
                    if cb not in call_tracker:
                        cb(tag, applied)
                        call_tracker.add(cb)


    def set_data(self, key, value):
        key = Path(key) if type(key) == str else key
        if not self.has_data(key) or value != self.get_data(key):
            self._callback_batch.add(key)
        super(EventModel, self).set_data(key, value)

    def remove_data(self, key):
        key = Path(key) if type(key) == str else key
        if self.has_data(key):
            self._callback_batch.add(key)
        super(EventModel, self).remove_data(key)

    def add_constraint(self, key, constraint):
        if key not in self._constraint_callback_batch:
            self._constraint_callback_batch[key] = set()
        
        if key in self.constraints:
            for s in self.constraints[key].expr.free_symbols:
                if s in self.constraint_callbacks and s not in constraint.expr.free_symbols:
                    for cb in self.constraint_callbacks[s]:
                        if cb not in self._constraint_discard_batch:
                            self._constraint_discard_batch[cb] = set()
                        self._constraint_discard_batch[cb].add(key)

        super(EventModel, self).add_constraint(key, constraint)
        for s in constraint.expr.free_symbols:
            if s in self.constraint_callbacks:
                self._constraint_callback_batch[key].update(self.constraint_callbacks[s])        

    def remove_constraint(self, key):    
        if key in self.constraints:
            if key not in self._constraint_callback_batch:
                self._constraint_callback_batch[key] = set()

            for s in self.constraints[key].expr.free_symbols:
                if s in self.constraint_callbacks:
                    self._constraint_callback_batch[key].update(self.constraint_callbacks[s])
        super(EventModel, self).remove_constraint(key)


    def clean_structure(self, until=2e9):
        super(EventModel, self).clean_structure(until)


    def register_on_operation_changed(self, pattern, callback):
        if self.__in_dispatch_mode:
            self.__callback_additions.append((pattern, callback))
        else:
            matcher = re.compile(pattern)
            if matcher not in self.tag_callbacks:
                self.tag_callbacks[matcher] = set()
            self.tag_callbacks[matcher].add(callback)

            if callback not in self._callback_matcher_registry:
                self._callback_matcher_registry[callback] = set()
            self._callback_matcher_registry[callback].add(matcher)


    def deregister_on_operation_changed(self, callback):
        if self.__in_dispatch_mode:
            self.__callback_removals.append(('o', callback))
        else:
            if callback in self._callback_matcher_registry:
                for m in self._callback_matcher_registry[callback]:
                    self.tag_callbacks[m].remove(callback)
                del self._callback_matcher_registry[callback]


    def register_on_model_changed(self, path, callback):
        if type(path) is str:
            path = Path(path)

        if self.__in_dispatch_mode:
            self.__callback_additions.append((path, callback))
        else:
            if path not in self.model_change_callbacks:
                self.model_change_callbacks[path] = set()
            self.model_change_callbacks[path].add(callback)
            
            if callback not in self._callback_path_registry:
                self._callback_path_registry[callback] = set()
            self._callback_path_registry[callback].add(path)


    def deregister_on_model_changed(self, callback):
        if self.__in_dispatch_mode:
            self.__callback_removals.append(('m', callback))
        else:
            if callback in self._callback_path_registry:
                for p in self._callback_path_registry[callback]:
                    self.model_change_callbacks[p].remove(callback)
                del self._callback_path_registry[callback]


    def register_on_constraints_changed(self, symbols, callback):
        if self.__in_dispatch_mode:
            self.__callback_additions.append((symbols, callback))
        else:
            for s in symbols:
                if s not in self.constraint_callbacks:
                    self.constraint_callbacks[s] = set()
                self.constraint_callbacks[s].add(callback)

            if callback not in self._callback_symbol_registry:
                self._callback_symbol_registry[callback] = set()
            self._callback_symbol_registry[callback].update(symbols)


    def deregister_on_constraints_changed(self, callback):
        if self.__in_dispatch_mode:
            self.__callback_removals.append(('c', callback))
        else:
            if callback in self._callback_symbol_registry:
                for s in self._callback_symbol_registry:
                    self.constraint_callbacks[s].remove(callback)
                del self._callback_symbol_registry[callback]


    def dispatch_events(self):
        self.__in_dispatch_mode = True

        # Model Updates
        call_tracker = set()

        for p in self._callback_batch:
            if p[:1] in self.model_change_callbacks:
                _dispatch_model_events(self.model_change_callbacks.get_sub_dict(p[:1]), 
                                       p[:1].get_data_no_throw(self.data_tree.data_tree),
                                       p[1:], call_tracker)

        # Constraint Updates
        for k, cbs in self._constraint_callback_batch.items():
            if len(cbs) > 0:
                c = self.constraints[k] if k in self.constraints else None
                for cb in cbs:
                    cb(k, c)
                    if cb in self._constraint_discard_batch:
                        self._constraint_discard_batch[cb].discard(k)

        for cb, keys in self._constraint_discard_batch.items():
            for k in keys:
                cb(k, None)

        # Clean-up
        self._callback_batch = PathSet()
        self._constraint_callback_batch = {}
        self._constraint_discard_batch  = {}

        self.__in_dispatch_mode = False

        for param, cb in self.__callback_additions:
            if isinstance(param, set):
                self.register_on_constraints_changed(param, cb)
            elif isinstance(param, Path):
                self.register_on_model_changed(param, cb)
            else:
                self.register_on_operation_changed(param, cb)

        for t, cb in self.__callback_removals:
            if t == 'o':
                self.deregister_on_operation_changed(cb)
            elif t == 'c':
                self.deregister_on_constraints_changed(cb)
            else:
                self.deregister_on_model_changed(cb)

        self.__callback_additions = []
        self.__callback_removals  = []