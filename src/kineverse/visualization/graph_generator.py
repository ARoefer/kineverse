import pydot

from kineverse.model.paths           import PathDict
from kineverse.model.history         import Timeline
from kineverse.model.articulation_model import ArticulationModel


def get_color(name, colors):
    for prefix, c in colors.items():
        if name[:len(prefix)] == prefix:
            return c
    return 'black'

def generate_modifications_graph(km, prefix_colors={}):
    tl = Timeline(km.get_data_chain())
    path_slots = PathDict()

    final_slots = []
    nodes = ['_final [label="Final Model", shape=box]']
    edges = []

    for node in reversed(tl):
        node_id_str = 't_{}'.format(node.stamp)
        nodes.append('{} [label="{}", color="{}"]'.format(node_id_str, node.tag, get_color(node.tag, prefix_colors)))
        print(node.inputs)
        for o in node.outputs:
            if o not in path_slots:
                final_slots.append(o)
                path_slots[o] = '_final:{}'.format(len(final_slots))
            edges.append('{}:{} -> {}'.format(node_id_str, str(o.to_symbol()), path_slots[o]))
        for i in node.inputs:
            path_slots[i] = '{}:{}'.format(node_id_str, str(i.to_symbol()))

    return 'digraph mod_graph {{rankdir=LR;\n    {};\n    {}\n}}'.format(';\n    '.join(nodes), ';\n    '.join(edges))

def generate_dependency_graph(km, prefix_colors={}):
    sorted_tags = sorted([(t, tag) for tag, t in km.timeline_tags.items()])
    nodes_str = ';\n    '.join(['t_{} [label="{}",color={}]'.format(t, tag, get_color(tag, prefix_colors)) for t, tag in sorted_tags])
    edges_str = ';\n    '.join([';\n    '.join(['t_{} -> t_{}'.format(d.stamp, c.stamp) for d in c.dependents]) for c in km.operation_history.chunk_history if len(c.dependents) > 0])
    return 'digraph dep_graph {{rankdir=LR\n    {};\n    {}\n}}'.format(nodes_str, edges_str)


def plot_graph(dot_graph, file):
    print(dot_graph)
    graphs = pydot.graph_from_dot_data(dot_graph)[0]
    if file[-4:].lower() == '.png':
        graphs.write_png(file)
    else:
        graphs.write_pdf(file)
