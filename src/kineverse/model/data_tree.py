from kineverse.json_serializable import JSONSerializable
from kineverse.gradients.gradient_math import spw
from kineverse.model.paths       import PathException

from yaml import load, dump

from multiprocessing import RLock


class DataTree(JSONSerializable):
    def __init__(self, parent=None):
        super(DataTree, self).__init__()

        self.parent = parent
        self.data_tree = {}
        self.value_table = {}
        # Mapping of {DLConcept: set}
        self.lock = RLock()

    def _json_data(self, json_dict):
        json_dict.update({'parent': self.parent,
                          'tree': self.data_tree, 
                          'values': {str(k): d for k, d in self.value_table.items()}})

    @classmethod
    def json_factory(cls, parent, tree, values):
        out = DataTree(parent)
        out.data_tree = tree
        out.value_table = {spw.Symbol(k.encode('utf-8')): v for k, v in values.items()}
        return out

    def __str__(self):
        return '{}\n{}'.format('\n'.join(['{}:\n  {}'.format(k, '\n  '.join(str(v).split('\n'))) for k, v in self.data_tree.items()]), str(self.parent))

    def __contains__(self, key):
        with self.lock:
            container = self.data_tree
            try:
                for part in key:
                    if type(container) is dict:
                        container = container[part]
                    elif type(container) is list:
                        container = container[int(part)]
                    else:
                        container = getattr(container, part)
                return True
            except (KeyError, IndexError, AttributeError):
                if self.parent is not None:
                    return key in self.parent
                return False

    def __setitem__(self, key, value):
        self.insert_data(key, value)

    def __getitem__(self, key):
        return self.find_data(key)

    def dump_to_file(self, filepath):
        stream = file(filepath, 'w')
        dump(self.id_map, stream)
        stream.close()

    def safe_find_data(self, key):
        with self.lock:
            return self.find_data(key)

    def safe_insert_data(self, key, data):
        with self.lock: 
            self.insert_data(key, data)

    def safe_remove_data(self, key):
        with self.lock:
            self.remove_data(key)

    def find_data(self, key):   
        return key.get_data(self.data_tree)

    def insert_data(self, key, data):
        try:
            container = key[:-1].get_data(self.data_tree)
        except PathException as e:
            raise Exception('Can not insert data at "{}", since attribute "{}" does not exist in object "{}".'.format(key, e.path[-1], e.path[:-1]))

        if type(container) is dict:
            is_new = key[-1] in container
            container[key[-1]] = data
        elif type(container) is list:
            idx = int(key[-1])
            is_new = idx == len(container)
            if is_new:
                container.append(data)
            else:
                container[idx] = data
        else:
            is_new = hasattr(container, key[-1])
            setattr(container, key[-1], data)


    def remove_data(self, key):
        try:
            container = key[:-1].get_data(self.data_tree)
        except PathException as e:
            raise Exception('Can not remove data at "{}", since attribute "{}" does not exist in object "{}".'.format(key, e.path[-1], e.path[:-1]))

        if type(container) is dict:
            if key[-1] in container:
                del container[key[-1]]
        elif type(container) is list:
            idx = int(key[-1])
            if idx < len(container):
                del container[idx]
        else:
            if hasattr(container, key[-1]):
                delattr(container, key[-1])         

    def get_data_map(self):
        return self.data_tree.copy()

    def __eq__(self, other):
        if isinstance(other, DataTree):
            return self.parent == other.parent and self.data_tree == other.data_tree and self.value_table == other.value_table
        return False