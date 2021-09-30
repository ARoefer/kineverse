import urdf_parser_py.urdf as urdf

from kineverse.utils import res_pkg_path

def hacky_urdf_parser_fix(urdf_str):
    # TODO this function is inefficient but the tested urdfs's aren't big enough for it to be a problem
    fixed_urdf = ''
    delete = False
    black_list = ['transmission', 'gazebo']
    black_open = ['<{}'.format(x) for x in black_list]
    black_close = ['</{}'.format(x) for x in black_list]
    for line in urdf_str.split('\n'):
        if len([x for x in black_open if x in line]) > 0:
            delete = True
        if len([x for x in black_close if x in line]) > 0:
            delete = False
            continue
        if not delete:
            fixed_urdf += line + '\n'
    return fixed_urdf

# Adds collision information to links if it is missing and they have visual information
def urdf_filler(model, fill_with_visual=True):
    for link in model.links:
        if hasattr(link, 'collisions'):
            if (link.collisions is None or len(link.collisions) == 0) and \
                                            (link.visuals is not None and \
                                               len(link.visuals) > 0) and fill_with_visual:
                for v in link.visuals:
                    link.add_aggregate('collision', urdf.Collision(v.geometry, v.origin))
        else:
            if link.collision is None and link.visual is not None:
                if fill_with_visual:
                    link.collision = urdf.Collision(link.visual.geometry, link.visual.origin)
                else:
                    link.collision = None
    return model

def load_urdf_str(string):
    return urdf_filler(urdf.URDF.from_xml_string(hacky_urdf_parser_fix(string)))

def load_urdf_file(path):
    with open(res_pkg_path(path), 'r') as f:
        return load_urdf_str(f.read())

