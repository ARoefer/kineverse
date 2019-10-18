import urdf_parser_py.urdf as urdf

# Adds collision information to links if it is missing and they have visual information
def urdf_filler(model):
    for link in model.links:
        if hasattr(link, 'collisions'):
            if (link.collisions is None or len(link.collisions) == 0) and (link.visuals is not None and len(link.visuals) > 0):
                for v in link.visuals:
                    link.add_aggregate('collision', urdf.Collision(v.geometry, v.origin))
        else:
            if link.collision is None and link.visual is not None:
                link.collision = urdf.Collision(link.visual.geometry, link.visual.origin)
    return model
