import pydot

from kineverse.model.kinematic_model import KinematicModel
from kineverse.model.kinematic_model import KinematicModel



def generate_modifications_graph(km):
    pass

def get_color(name, colors):
    for prefix, c in colors.items():
        if name[:len(prefix)] == prefix:
            return c
    return 'black'

def generate_dependency_graph(km, prefix_colors={}):
    sorted_tags = sorted([(t, tag) for tag, t in km.timeline_tags.items()])
    nodes_str = ';\n    '.join(['t_{} [label="{}",color={}]'.format(t, tag, get_color(tag, prefix_colors)) for t, tag in sorted_tags])
    edges_str = ';\n    '.join([';\n    '.join(['t_{} -> t_{}'.format(d.stamp, c.stamp) for d in c.dependents]) for c in km.operation_history.chunk_history if len(c.dependents) > 0])
    return 'digraph dep_graph {{\n    {};\n    {}\n}}'.format(nodes_str, edges_str)


def plot_graph(dot_graph, file):
    graphs = pydot.graph_from_dot_data(dot_graph)
    if file[-4:].lower() == '.png':
        graphs.write_png(file)
    else:
        graphs.write_pdf(file)
