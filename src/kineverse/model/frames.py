from kineverse.gradients.gradient_math import spw

class Frame(object):
    def __init__(self, parent_path, pose=None, to_parent=None):
        self.parent    = parent_path
        self.pose      = pose if pose is not None else spw.eye(4)
        self.to_parent = to_parent if to_parent is not None else self.pose

    def __copy__(self):
        return Frame(self.parent, self.pose)

    def __deepcopy__(self, memo):
        out = Frame(self.parent, self.pose * 1)
        memo[id(self)] = out
        return out

class Transform(object):
    def __init__(self, from_frame, to_frame, pose):
        self.from_frame = from_frame
        self.to_frame   = to_frame
        self.pose       = pose

    def __copy__(self):
        return Transform(self.from_frame, self.to_frame, self.pose)

    def __deepcopy__(self, memo):
        out = Transform(self.from_frame, self.to_frame, self.pose * 1)
        memo[id(self)] = out
        return out