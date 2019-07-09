from yaml import load, dump

from multiprocessing import RLock


class DataTree(object):
	def __init__(self, parent=None):
		super(DataTree, self).__init__()

		self.parent = parent
		self.data_tree = {}
		self.value_table = {}
		# Mapping of {DLConcept: set}
		self.lock = RLock()

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

	def find_data(self, key):	
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
				return container
			except (KeyError, IndexError, AttributeError):
				if self.parent is not None:
					return self.parent.find_data(key)
				raise Exception('Unknown data id "{}"'.format(key))

	def insert_data(self, key, data):
		with self.lock:	
			container = self.data_tree
			try:
				for part in key[:-1]:
					if type(container) is dict:
						container = container[part]
					elif type(container) is list:
						container = container[int(part)]
					else:
						container = getattr(container, part)
			except (KeyError, IndexError, AttributeError):
				raise Exception('Can not insert data at "{}", "{}" does not exist.'.format(key, part))

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
		with self.lock:
			container = self.data_tree
			try:
				for part in key[:-1]:
					if type(container) is dict:
						container = container[part]
					elif type(container) is list:
						container = container[int(part)]
					else:
						container = getattr(container, part)
			except (KeyError, IndexError, AttributeError):
				raise Exception('Can not remove data at "{}", "{}" does not exist.'.format(key, part))

			if type(container) is dict:
				del container[key[-1]]
			elif type(container) is list:
				idx = int(key[-1])
				del container[idx]
			else:
				delattr(container, key[-1])			

	def get_data_map(self):
		return self.data_tree.copy()