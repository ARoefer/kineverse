from kineverse.json_wrapper            import JSONSerializable
from kineverse.gradients.gradient_math import spw

class Frame(JSONSerializable):
    def __init__(self, parent_path, pose=None, to_parent=None):
        self.parent    = parent_path
        self.pose      = pose if pose is not None else spw.eye(4)
        self.to_parent = to_parent if to_parent is not None else self.pose

    def _json_data(self, json_dict):
        json_dict.update({'parent_path': self.parent, 
                          'pose': self.pose, 
                          'to_parent': self.to_parent})

    def __copy__(self):
        return Frame(self.parent, self.pose)

    def __deepcopy__(self, memo):
        out = Frame(self.parent, self.pose * 1)
        memo[id(self)] = out
        return out

class Transform(JSONSerializable):
    def __init__(self, from_frame, to_frame, pose):
        self.from_frame = from_frame
        self.to_frame   = to_frame
        self.pose       = pose

    def _json_data(self, json_dict):
        json_dict.update({'from_frame': self.from_frame,
                          'to_frame':   self.to_frame,
                          'pose':       self.pose})

    def __copy__(self):
        return Transform(self.from_frame, self.to_frame, self.pose)

    def __deepcopy__(self, memo):
        out = Transform(self.from_frame, self.to_frame, self.pose * 1)
        memo[id(self)] = out
        return out