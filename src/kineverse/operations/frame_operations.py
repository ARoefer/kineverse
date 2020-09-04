import kineverse.gradients.common_math as cm

from kineverse.gradients.gradient_math     import inverse_frame, dot
from kineverse.model.data_tree             import DataTree
from kineverse.model.frames                import Frame, Transform
from kineverse.model.articulation_model    import ArticulationModel
from kineverse.model.paths                 import Path, collect_paths
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
    def init(self, path, frame):
        self.frame = frame
        attrs = collect_paths(self.frame, Path('frame'))
        super(CreateRelativeFrame, self).init('Frame',
                                              [str(a) for a in attrs],
                                              parent=Path(self.frame.parent),
                                              **{str(a): path + a[1:] for a in attrs})
    def _apply(self, ks, parent):
        return {'frame': Frame(self.frame.parent, dot(parent.pose, self.frame.to_parent), self.frame.to_parent)}, {}


class CreateRelativeTransform(Operation):
    def init(self, transform_path, from_frame, to_frame):
        self.tf_obj = Transform(from_frame, to_frame, None)
        attrs       = collect_paths(self.tf_obj, Path('transform'))
        super(CreateRelativeTransform, self).init('Relative Transform',
                                                  [str(a) for a in attrs],
                                                  frame_a=from_frame,
                                                  frame_b=to_frame,
                                                  **{str(a): transform_path + a[1:] for a in attrs})

    def _apply(self, ks, frame_a, frame_b):
        return {'transform': Transform(self.tf_obj.from_frame, 
                                       self.tf_obj.to_frame, 
                                       fk_a_in_b(ks, frame_a, frame_b))}, {}
