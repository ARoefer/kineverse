import symengine as sp
import yaml
from kineverse.type_sets import symengine_matrix_types, symengine_types, GC, GM

loaders = [yaml.BaseLoader, yaml.SafeLoader, yaml.FullLoader, yaml.UnsafeLoader]

def nested_list_to_sym(l):
    if type(l) == list:
        return [nested_list_to_sym(x) for x in l]
    else:
        return sp.sympify(l)

def nested_symlist_to_yaml(l):
    if type(l) == list:
        return [nested_symlist_to_yaml(x) for x in l]
    else:
        return '{}'.format(str(l))

def yaml_matrix_representer(dumper, matrix):
    return dumper.represent_sequence('!SymMatrix', nested_symlist_to_yaml(matrix.tolist()))

def yaml_matrix_constructor(loader, node):
    return sp.Matrix(nested_list_to_sym(loader.construct_sequence(node, deep=True)))

def yaml_sym_representer(dumper, data):
    return dumper.represent_scalar('!SymExpr', str(data))

def yaml_sym_constructor(loader, node):
    return sp.sympify(str(loader.construct_scalar(node)))

def yaml_wrap_symengine_types():
    for loader in loaders:
        yaml.add_constructor('!SymExpr',   yaml_sym_constructor, Loader=loader)
        yaml.add_constructor('!SymMatrix', yaml_matrix_constructor, Loader=loader)

    for t in symengine_types:
        if t in symengine_matrix_types:
            yaml.add_representer(t, yaml_matrix_representer)
        else:
            yaml.add_representer(t, yaml_sym_representer)


yaml_wrap_symengine_types()

def yaml_gc_representer(dumper, gc):
    return dumper.represent_mapping('!GradientContainer', {'expr': gc.expr, 'diffs': gc.gradients})

def yaml_gc_constructor(loader, node):
    pdict = loader.construct_mapping(node, deep=True)
    return GC(sp.sympify(pdict['expr']), {s: e for s, e in pdict['diffs'].items()})

def yaml_gm_representer(dumper, gm):
    return dumper.represent_sequence('!GradientMatrix', gm.expr)

def yaml_gm_constructor(loader, node):
    return GM(loader.construct_sequence(node, deep=True))


yaml.add_representer(GC, yaml_gc_representer)
yaml.add_representer(GM, yaml_gm_representer)

for loader in loaders:
    yaml.add_constructor('!GradientContainer', yaml_gc_constructor, Loader=loader)
    yaml.add_constructor('!GradientMatrix',    yaml_gm_constructor, Loader=loader)