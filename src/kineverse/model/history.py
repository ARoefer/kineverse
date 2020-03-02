from sortedcontainers import SortedList, SortedSet

from kineverse.model.paths import Path

class Timeline(SortedList):
    def get_floor(self, key):
        first = 0
        last  = len(self) - 1
        found = None
        pos   = -1

        while first <= last:
            pos   = (first + last) // 2
            found = self[pos] if self[pos] <= key else found
            if self[pos] == key:
                break

            if key < self[pos]:
                last  = pos - 1
            else:
                first = pos + 1
        return pos, found

    def get_ceil(self, key):
        first = 0
        last  = len(self) - 1
        found = None
        pos   = -1

        while first <= last:
            pos   = (first + last) // 2
            found = self[pos] if self[pos] >= key else found
            if self[pos] == key:
                break

            if key < self[pos]:
                last  = pos - 1
            else:
                first = pos + 1
        return pos, found


class History(object):
    def __init__(self, history=None, modification_history=None):
        # Dict var_name -> Timeline
        self.chunk_history = Timeline() if history is None else Timeline(history)
        if modification_history is None:
            self.modification_history = {}
            for c in self.chunk_history:
                for p in c.modifications:
                    if p not in self.modification_history:
                        self.modification_history[p] = Timeline()
                    self.modification_history[p].add(c)
                for p in c.dependencies:
                    if p not in self.modification_history:
                        raise Exception('Illegal sequence of operations was supplied! Referenced dependency {} does not exist at time {}'.format(p, c.stamp))
                    self.modification_history[p][-1].dependents.add(c)
        else:
            self.modification_history = modification_history
        self.dirty_chunks  = SortedSet()

    def __iter__(self):
        return iter(self.chunk_history)

    def __len__(self):
        return len(self.modification_history)

    def get_time_stamp(self, before=None, after=None):
        if before is not None:
            pos, succ = self.chunk_history.get_ceil(before) if type(before) != Chunk else self.chunk_history.get_ceil(before.stamp)
            return 0.5 * (succ.stamp + self.chunk_history[pos - 1].stamp) if pos > 0 else succ.stamp - 1
        elif after is not None:
            pos, succ = self.chunk_history.get_floor(after) if type(after) != Chunk else self.chunk_history.get_floor(after.stamp)
            return 0.5 * (succ.stamp + self.chunk_history[pos + 1].stamp) if pos < len(self.chunk_history) - 1 else succ.stamp + 1
        return self.chunk_history[-1].stamp + 1 if len(self.chunk_history) > 0 else 1

    @profile
    def _insert_modification(self, chunk, path):
            if path not in self.modification_history:
                self.modification_history[path] = Timeline()
            _, pred = self.modification_history[path].get_floor(chunk.stamp)
            if pred is not None:
                to_remove = set()
                for d in pred.dependents:
                    # Fetch all dependents from predecessor which are going to depend on the new chunk
                    # Save them as dependents and mark them as dirty
                    if d.stamp > chunk.stamp:
                        dep_overlap_diff = d.dependencies.difference(chunk.modifications)
                        # Is there at least one element overlap
                        if len(dep_overlap_diff) < len(d.dependencies):  
                            chunk.dependents.add(d)
                            self.dirty_chunks.add(d)
                            # If there is no remaining overlap with pred anymore, remove d
                            if len(dep_overlap_diff.difference(pred.modifications)) == len(dep_overlap_diff):
                                to_remove.add(d)
                pred.dependents -= to_remove
            self.modification_history[path].add(chunk)

    @profile
    def insert_chunk(self, chunk):
        for p in chunk.dependencies:
            if p not in self.modification_history:
                raise Exception('Chunk depends on attribute without history!\n Operation "{}" at {}\n Attribute: {}\n'.format(chunk.operation.name, chunk.stamp, p))
            _, pred = self.modification_history[p].get_floor(chunk.stamp)
            if pred is None:
                raise Exception('Chunk at time {} executing "{}" depends on attributes with empty history! Attributes:\n  {}'.format(chunk.stamp, chunk.operation.name, '\n  '.join([str(p) for p in chunk.dependencies if p not in self.modification_history or self.modification_history[p].get_floor(chunk.stamp)[1] is None])))
            pred.dependents.add(chunk)

        for p in chunk.modifications:
            self._insert_modification(chunk, p)

        self.chunk_history.add(chunk)

    @profile
    def remove_chunk(self, chunk):
        for p in chunk.modifications:
            if self.modification_history[p][0] == chunk and len(chunk.dependents) > 0 and max([p in c.dependencies for c in chunk.dependents]):
                raise Exception('Can not remove chunk at timestamp {} because it is the founding chunk in the history of {} and would create dangling dependencies.'.format(chunk.stamp, p))
        
        for p in chunk.modifications:
            self.modification_history[p].discard(chunk)
            _, pred = self.modification_history[p].get_floor(chunk.stamp)
            # Copy dependents that depend on this variable to predecessor
            if pred is not None:
                pred.dependents.update({d for d in chunk.dependents if p in d.dependencies})

        for p in chunk.dependencies:
            pos, pred = self.modification_history[p].get_floor(chunk.stamp)
            if pred is None:
                raise Exception('Chunk depends on attribute with empty history!')
            # It can happen that this chunk modifies the variable it depends on. 
            # In this case it needs to be removed from the history and from 
            if pred == chunk:
                pos  -= 1
                pred  = self.modification_history[p][pos]
            pred.dependents.discard(chunk)

        self.chunk_history.remove(chunk)
        self.dirty_chunks.update(chunk.dependents)

    @profile
    def replace_chunk(self, c_old, c_new):
        if c_old.stamp != c_new.stamp:
            raise Exception('Can only replace chunk if stamps match. Stamps:\n Old: {:>8.3f}\n New: {:>8.3f}'.format(c_old.stamp, c_new.stamp))

        overlap = c_old.modifications.intersection(c_new.modifications)
        if len(overlap) != len(c_old.modifications):
            raise Exception('Chunks can only be replaced by others with at least the same definition coverage. Missing variables:\n {}'.format('\n '.join(sorted(c_old.modifications.difference(c_new.modifications)))))

        new_deps = {p: self.modification_history[p].get_floor(c_new.stamp)[1] if p in self.modification_history else None for p in c_new.dependencies.difference(overlap)}
        if None in new_deps.values():
            raise Exception('Replacement chunk at {} tries to depend on variables with insufficient histories. variables:\n {}'.format('\n '.join(sorted(new_deps.keys()))))

        for p in overlap:
            pos, _ = self.modification_history[p].get_floor(c_old.stamp)
            # If we are already here, we might as well remove old and establish new deps
            if p in c_old.dependencies:
                self.modification_history[p][pos - 1].dependents.discard(c_old)
            if p in c_new.dependencies:
                self.modification_history[p][pos - 1].dependents.add(c_new)
            self.modification_history[p].remove(c_old)
            self.modification_history[p].add(c_new)
        
        c_new.dependents = c_old.dependents.copy()
        self.flag_dirty(*c_new.dependents)

        # Remove old, non-modified deps
        for p in c_old.dependencies.difference(overlap):
            self.modification_history[p].get_floor(c_old.stamp)[1].dependents.remove(c_old)

        # Insert additional modifications
        for p in c_new.modifications.difference(overlap):
            self._insert_modification(c_new, p)

        for c in new_deps.values():
            c.dependents.add(c_new)

        self.chunk_history.remove(c_old)
        self.chunk_history.add(c_new)


    def get_chunk_by_index(self, idx):
        return self.chunk_history[idx]

    def get_chunk(self, stamp):
        return self.get_chunk_pos(stamp)[0]

    def get_chunk_pos(self, stamp):
        pos, chunk = self.chunk_history.get_floor(stamp)
        return (chunk, pos) if chunk is None or chunk.stamp == stamp else (None, None)

    def flag_dirty(self, *chunks):
        self.dirty_chunks.update(chunks)

    def flag_clean(self, *chunks):
        for c in chunks:
            self.dirty_chunks.discard(c)

    def expand_dirty_set(self):
        active_set = set(self.dirty_chunks)
        while len(active_set) > 0:
            a = active_set.pop()
            u = a.dependents.difference(self.dirty_chunks)
            active_set.update(u)
            self.dirty_chunks.update(u)

    def get_dirty(self):
        return self.dirty_chunks.copy()

    def get_subhistory(self, time):
        if len(self.chunk_history) > 0 and self.chunk_history[0].stamp >= time:
            chunks      = self.chunk_history[:self.chunk_history.get_floor(time)[0] + 1]
            mod_history = {p: Timeline(h[:h.get_floor(time)]) for p, h in self.modification_history.items() if h[0].stamp >= time}
            return History(chunks, mod_history)
        return History()

    def get_history_of(self, *paths):
        out = set()
        remaining = set()
        for p in paths:
            if p in self.modification_history:
                remaining.update(self.modification_history[p])

        while len(remaining) > 0:
            chunk = remaining.pop()
            out.add(chunk)
            for p in chunk.dependencies:
                pos, dep = self.modification_history[p].get_floor(chunk.stamp)
                if dep == chunk: # Catch if predecessor is chunk itself
                    dep = self.modification_history[p][pos - 1]
                if dep not in out:
                    remaining.add(dep)

        return Timeline(out)



    def str_history_of(self, p):
        if p not in self.modification_history:
            raise Exception('Path {} has no history.'.format(p))
        return '\n'.join(['{:>8.3f} : {}'.format(chunk.stamp, str(chunk.op)) for chunk in self.modification_history[p]])

    def str_history(self):
        return '\n'.join(['{:>8.3f} : {}'.format(chunk.stamp, str(chunk.op)) for chunk in self.chunk_history])

    def __eq__(self, other):
        if isinstance(other, History):
            return self.chunk_history == other.chunk_history
        return False


class StampedData(object):
    def __init__(self, stamp, **kwargs):
        self.stamp = stamp
        for k, v in kwargs.items():
            setattr(self, k, v)

    def __lt__(self, other):
        if isinstance(other, StampedData):
            return self.stamp < other.stamp
        return self.stamp < other

    def __gt__(self, other):
        if isinstance(other, StampedData):
            return self.stamp > other.stamp
        return self.stamp > other

    def __cmp__(self, other):
        if isinstance(other, StampedData):
            return cmp(self.stamp, other.stamp)
        return cmp(self.stamp, other)

    def __str__(self):
        return 'Stamped Data {}'.format(self.stamp)

    def __eq__(self, other):
        if isinstance(other, StampedData):
            return self.stamp == other.stamp
        return False


class Chunk(StampedData):
    def __init__(self, stamp, op):
        super(Chunk, self).__init__(stamp,
                                    operation=op,
                                    dependencies={p for p in op.args_paths.values() if type(p) == Path},
                                    modifications={p for p in op.mod_paths.values()},
                                    dependents=set())

    def __str__(self):
        return 'Stamp: {}\n Arguments: {}\n Modifications: {}\n Dependents: {}\n'.format(self.stamp, ', '.join(self.dependencies), ', '.join(self.modifications), ', '.join([str(d.stamp) for d in self.dependents]))

    def __eq__(self, other):
        if isinstance(other, Chunk):
            return super(Chunk, self).__eq__(other) and self.operation == other.operation and self.dependencies == other.dependencies and self.modifications == other.modifications
        return False
