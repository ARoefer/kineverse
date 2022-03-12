import kineverse.gradients.common_math as cm

from kineverse.gradients.gradient_math     import inverse_frame, dot
from kineverse.model.data_tree             import DataTree
from kineverse.model.frames                import Frame, Transform
from kineverse.model.articulation_model    import ArticulationModel
from kineverse.model.paths                 import Path, CPath, collect_paths
from kineverse.operations.operation        import Operation

def collect_chain(ks, frame):
    out    = []
    while True:
        out.append(frame)
        if ks.has_data(frame.parent):
            frame = ks.get_data(frame.parent)
        else:
            break
    out.reverse()
    return out

def fk_a_in_b(ks, frame_a, frame_b):
    tf_chain_a = collect_chain(ks, frame_a)
    tf_chain_b = collect_chain(ks, frame_b)

    if tf_chain_a[0] != tf_chain_b[0] and tf_chain_a[0].parent != tf_chain_b[0].parent:
        raise Exception('Frames "{}" and "{}" have no common root!'.format(frame_a, frame_b))

    if frame_b in tf_chain_a:
        return dot(*[f.to_parent for f in tf_chain_a[tf_chain_a.index(frame_b) + 1:]])
    elif frame_a in tf_chain_b:
        return inverse_frame(dot(*[f.to_parent for f in tf_chain_b[tf_chain_b.index(frame_a) + 1:]]))
    else:
        x = 0
        while tf_chain_a[x] == tf_chain_b[x]:
            x += 1

        a_in_root = dot(*[f.to_parent for f in tf_chain_a[x:]])
        b_in_root = dot(*[f.to_parent for f in tf_chain_b[x:]])

        return dot(inverse_frame(b_in_root), a_in_root)

class CreateRelativeFrame(Operation):
    def __init__(self, path, parent_path, child_frame):
        super(CreateRelativeFrame, self).__init__({'output': path}, 
                                                  parent_frame=Path(parent_path), 
                                                  parent_path=CPath(parent_path),
                                                  child_frame=child_frame)
        self._save_serialization_args(path, parent_path, child_frame)

    def _execute_impl(self, parent_path, parent_frame, child_frame):
        self.output = Frame(parent_path, 
                            dot(parent_frame.pose, child_frame.to_parent), 
                            child_frame.to_parent)
        self.constraints = {}


class CreateRelativeTransform(Operation):
    def __init__(self, path, from_frame_path, to_frame_path):
        super(CreateRelativeTransform, self).__init__({'output': path},
                                                      from_frame=Path(from_frame_path),
                                                      to_frame=Path(to_frame_path),
                                                      ff_path=CPath(from_frame_path),
                                                      tf_path=CPath(to_frame_path))
        self._save_serialization_args(path, from_frame_path, to_frame_path)

    # TODO: Inefficient
    def _execute_impl(self, from_frame, to_frame, ff_path, tf_path):
        self.output = Transform(ff_path, 
                                tf_path, 
                                dot(inverse_frame(to_frame.pose), from_frame.pose))
        self.constraints = {}
