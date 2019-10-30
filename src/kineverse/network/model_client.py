import rospy

from kineverse.model.event_model      import EventModel
from kineverse.model.kinematic_model  import Constraint
from kineverse.model.paths            import Path, PathSet
from kineverse.network.ros_conversion import json, decode_constraint_filtered
from kineverse.time_wrapper           import Time

from kineverse.msg import ModelUpdate as ModelUpdateMsg

from kineverse.srv import GetModel       as GetModelSrv
from kineverse.srv import GetConstraints as GetConstraintsSrv




class ModelClient(object):
    """Client for mirroring a complete or partial kinematic model."""
    def __init__(self, model_type, *args):
        if model_type is not None and not issubclass(model_type, EventModel):
            raise Exception('Model type must be a subclass of EventModel. Type "{}" is not.'.format(model_type))

        self.km = model_type(*args) if model_type is not None else EventModel()

        rospy.wait_for_service('get_model')
        self.srv_get_model = rospy.ServiceProxy('get_model', GetModelSrv)

        rospy.wait_for_service('get_constraints')
        self.srv_get_constraints = rospy.ServiceProxy('get_constraints', GetConstraintsSrv)

        self.sub_model = rospy.Subscriber('/km', ModelUpdateMsg, self.cb_model_update, queue_size=1)
        self._last_update    = Time(0)
        self.tracked_paths   = PathSet()
        self.tracked_symbols = set()
        for n, a in [(d, getattr(self.km, d)) for d in dir(self.km) if d[0] != '_']:
            if callable(a) and not hasattr(self, n):
                setattr(self, n, a)


    def kill(self):
        pass

    def has_data(self, path):
        if type(path) == str:
            path = Path(path)

        if path not in self.tracked_paths: 
            self._start_tracking(path)
        return self.km.has_data(path)


    def get_data(self, path):
        if type(path) == str:
            path = Path(path)

        if path not in self.tracked_paths: 
            self._start_tracking(path)
        return self.km.get_data(path)


    def _update_model(self, msg):
        if len(msg.paths) != len(msg.data):
            raise Exception('Message path and data arrays need to be of equal length.\n  Paths: {}\n   Data: {}'.format(len(msg.paths), len(msg.data)))

        for str_path, data in zip(msg.paths, msg.data):
            path = Path(str_path)
            if path in self.tracked_paths:
                print('Updating {}'.format(path))
                self.km.set_data(path, json.loads(data))

        for cmsg in msg.constraints:
            c = decode_constraint_filtered(cmsg, self.tracked_symbols)
            if c is not None:
                self.km.add_constraint(cmsg.name, c)
            elif self.km.has_constraint(cmsg.name):
                self.km.remove_constraint(cmsg.name)


    def cb_model_update(self, msg):
        if msg.stamp < self._last_update:
            return

        self._last_update = msg.stamp

        self._update_model(msg.update)

        for p in msg.deleted_paths:
            self.km.remove_data(p)


        for k in msg.deleted_constraints:
            self.km.remove_constraint(k)

        self.km.clean_structure()
        self.km.dispatch_events()


    def get_constraints_by_symbols(self, symbol_set):
        if len(symbol_set.intersection(self.tracked_symbols)) < len(symbol_set):
            diff = symbol_set.difference(self.tracked_symbols)
            n_const = self.srv_get_constraints([str(s) for s in diff]).constraints
            for cmsg in n_const:
                self.km.add_constraint(cmsg.name, decode_constraint_filtered(cmsg, diff))
            self.tracked_symbols |= diff
        return self.km.get_constraints_by_symbols(symbol_set)


    def _start_tracking(self, path):
        print('Started tracking {}'.format(path))
        self.tracked_paths.add(path)
        self._update_model(self.srv_get_model([str(path)],[]).model)
        self.km.clean_structure()
        self.km.dispatch_events()


    def register_on_model_changed(self, path, callback):
        if type(path) == str:
            path = Path(path)

        self.km.register_on_model_changed(path, callback)
        if path not in self.tracked_paths: 
            self._start_tracking(path)

    def deregister_on_model_changed(self, callback):
        self.km.deregister_on_model_changed(callback)    

    def register_on_constraints_changed(self, symbols, callback):
        self.km.register_on_constraints_changed(symbols, callback)
        self.tracked_symbols |= symbols

    def deregister_on_constraints_changed(self, callback):
        self.km.deregister_on_constraints_changed(callback)