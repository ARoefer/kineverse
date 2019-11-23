import traceback

import simplejson as json
import symengine  as sp

from kineverse.json_serializable import JSONSerializable
from kineverse.type_sets         import symengine_matrix_types, symengine_types, is_symbolic
from kineverse.utils             import import_class

json_sym_matrix = 'SYM_MATRIX'
json_sym_expr   = 'SYM_EXPR'

class_registry = {}


class KineverseJSONEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, JSONSerializable):
            return obj.json_data()
        elif type(obj) in symengine_matrix_types:
            return {'__type__': json_sym_matrix, 'data': obj.tolist()}
        elif type(obj) in symengine_types:
            return {'__type__': json_sym_expr,   'data': str(obj)} if is_symbolic(obj) else float(obj)
        else:
            try:
                return super(KineverseJSONEncoder, self).default(obj)
            except TypeError as e:
                raise TypeError('JSON serialization failed while encoding object "{}" of type "{}"'.format(str(obj), type(obj)))


    def nested_symlist_to_json(self, l):
        if type(l) == list:
            return [self.nested_symlist_to_json(x) for x in l]
        else:
            return {'__type__': json_sym_expr, 'data': str(l)}


def nested_list_to_sym(l):
    if type(l) == list:
        return [nested_list_to_sym(x) for x in l]
    else:
        return sp.sympify(l.encode('utf-8')) if isinstance(l, unicode) else sp.sympify(l)


def json_decoder(dct):
    if '__type__' in dct:
        t = dct['__type__']
        if t == json_sym_matrix:
            return sp.Matrix(dct['data'])# nested_list_to_sym(dct['data'])
        elif t == json_sym_expr:
            try:
                return sp.sympify(dct['data'].encode('utf-8')) if isinstance(dct['data'], unicode) else sp.sympify(dct['data'])
            except NameError as e:
                raise Exception('NameError Occured while trying to sympify string {}. Error: {}\n{}'.format(repr(dct['data']), str(e), traceback.format_exc()))
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