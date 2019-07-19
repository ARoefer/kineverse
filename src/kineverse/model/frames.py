class Frame(object):
    def __init__(self, parent_path, pose, to_parent=None):
        self.parent    = parent_path
        self.to_parent = pose if to_parent is None else to_parent
        self.pose      = pose

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