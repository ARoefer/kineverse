import urdf_parser_py.urdf as urdf

# Adds collision information to links if it is missing and they have visual information
def urdf_filler(model):
    for link in model.links:
        if link.collision is None and link.visual is not None:
            link.collision = urdf.Collision(link.visual.geometry, link.visual.origin)
    return model
