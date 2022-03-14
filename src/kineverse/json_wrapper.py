from importlib import import_module
import traceback

import kineverse.gradients.common_math as cm
import simplejson as json

if cm.SYM_MATH_ENGINE == 'SYMENGINE':
    import symengine  as sp
    sym_parser = sp.sympify
elif cm.SYM_MATH_ENGINE == 'CASADI':
    from kineverse.casadi_parser import parse as parse_casadi
    sym_parser = parse_casadi

from kineverse.json_serializable import JSONSerializable
from kineverse.utils             import import_class

json_sym_matrix = 'SYM_MATRIX'
json_sym_expr   = 'SYM_EXPR'
json_fn         = 'FUNCTION'

class_registry = {}
function_registry = {}

def encode_symbolic_number(obj):
    return float(obj) if type(obj) in cm.symfloats else int(obj)

class KineverseJSONEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, JSONSerializable):
            return obj.json_data()
        elif type(obj) in cm.matrix_types:
            return {'__type__': json_sym_matrix, 'data': str(obj)}
        elif type(obj) in cm.symfloats:
            if cm.is_symbolic(obj):
                return {'__type__': json_sym_expr, 'data': str(obj)}
            return encode_symbolic_number(obj)
        elif type(obj) == type(encode_symbolic_number) or type(obj) == type(KineverseJSONEncoder):
            return {'__type__': json_fn, 
                    'module': f'{obj.__module__}', 
                    'name': obj.__qualname__}
        else:
            try:
                return super(KineverseJSONEncoder, self).default(obj)
            except TypeError as e:
                raise TypeError('JSON serialization failed while encoding object "{}" of type "{}"'.format(str(obj), type(obj)))

    @staticmethod
    def nested_symlist_to_json(l):
        if type(l) == list:
            return [KineverseJSONEncoder.nested_symlist_to_json(x) for x in l]
        else:
            return str(l)


def nested_list_to_sym(l):
    if type(l) == list:
        return [nested_list_to_sym(x) for x in l]
    else:
        return sym_parser(l)


def json_decoder(dct):
    if '__type__' in dct:
        t = dct['__type__']
        if t == json_sym_matrix:
            return sym_parser(dct['data'])
        elif t == json_sym_expr:
            try:
                return sym_parser(dct['data'])
            except NameError as e:
                raise Exception('NameError Occured while trying to sympify string {}. Error: {}\n{}'.format(repr(dct['data']), str(e), traceback.format_exc()))
        elif t == json_fn:
            key = (dct['module'], dct['name'])
            if key not in function_registry:
                m = import_module(key[0])
                f = m
                try:
                    for n in key[1].split('.'):
                        f = getattr(f, n)
                except AttributeError as e:
                    raise TypeError(f'Failed to deserialze function "{key[0]}.{key[1]}": {e}')
                if type(f) is not type(json_decoder) and type(f) is not type:
                    raise TypeError(f'Deserialized function "{key[0]}.{key[1]}" is not a function but a {type(f)}')
                function_registry[key] = f
            return function_registry[key]
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
