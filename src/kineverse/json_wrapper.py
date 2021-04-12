import traceback

import kineverse.gradients.gradient_math as gm
import simplejson as json
import symengine  as sp

from kineverse.json_serializable import JSONSerializable
from kineverse.utils             import import_module, import_class

class_registry  = {}
module_registry = {}

class KineverseJSONEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, JSONSerializable):
            return obj.json_data()
        elif type(obj) in gm.math_types:
            return gm.serialize_json(obj)
        elif type(obj) == type(import_class):
            return {'__type__': 'PY_FUNCTION', 
                    'module': str(obj.__module__), 
                    'name': obj.__name__}
        elif type(obj) == type:
            return {'__type__': 'PY_TYPE', 'data': str(obj)}
        else:
            try:
                return super(KineverseJSONEncoder, self).default(obj)
            except TypeError as e:
                raise TypeError('JSON serialization failed while encoding object "{}" of type "{}"'.format(str(obj), type(obj)))


def json_decoder(dct):
    if '__type__' in dct:
        t = dct['__type__']
        if t in gm.JSON_TRIGGERS:
            return gm.deserialize_json(dct)
        elif t == 'PY_TYPE':
            t_name = dct['data'][8:-2]
            if t_name not in class_registry:
                class_registry[t_name] = import_class(t_name)
            return class_registry[t_name]
        elif t == 'PY_FUNCTION':
            t_name = dct['module']
            if t_name not in module_registry:
                module_registry[t_name] = import_module(t_name)
            return getattr(module_registry[t_name], dct['name'])
        elif t[:8] == '<class \'' and t[-2:] == '\'>':
            t = t[8:-2]
            if t not in class_registry:
                class_registry[t] = import_class(t)
            t = class_registry[t]
            if not issubclass(t, JSONSerializable):
                raise Exception('Can not decode JSON object tagged with type "{}". The type is not a subtype of JSONSerializable'.format(t))
            try:
                if hasattr(t, 'json_factory'):
                    return t.json_factory(**{k: v for k, v in dct.items() if k != '__type__'})
                return t(**{k: v for k, v in dct.items() if k != '__type__'})
            except TypeError as e:
                raise TypeError('Error occurred while instantiating an object of type "{}":\n  {}'.format(t.__name__, e))
    return dct


def load(fp):
    return json.load(fp, object_hook=json_decoder)

def loads(string):
    return json.loads(string, object_hook=json_decoder)

def dumps(obj, **kwargs):
    return KineverseJSONEncoder(for_json=True, **kwargs).encode(obj)

def dump(obj, fp, **kwargs):
    json.dump(obj, fp, cls=KineverseJSONEncoder, for_json=True, check_circular=True, **kwargs)
