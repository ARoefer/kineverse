import kineverse.gradients.common_math as cm

from kineverse.model.paths             import Path
from kineverse.json_wrapper            import JSONSerializable

class Frame(JSONSerializable):
    def __init__(self, parent_path, pose=None, to_parent=None):
        self.parent    = parent_path
        self.pose      = pose if pose is not None else cm.eye(4)
        self.to_parent = to_parent if to_parent is not None else self.pose

    def _json_data(self, json_dict):
        json_dict.update({'parent_path': self.parent, 
                          'pose': self.pose, 
                          'to_parent': self.to_parent})

    def __copy__(self):
        return Frame(self.parent, self.pose, self.to_parent)

    def __deepcopy__(self, memo):
        out = Frame(self.parent, self.pose * 1, self.to_parent * 1)
        memo[id(self)] = out
        return out

    def __eq__(self, other):
        if isinstance(other, Frame):
            return self.parent == other.parent and self.pose == other.pose and self.to_parent == other.to_parent
        return False


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
        return Transform(self.from_frame, self.to_frame, self.pose * 1)

    def __deepcopy__(self, memo):
        out = Transform(self.from_frame, self.to_frame, self.pose * 1)
        memo[id(self)] = out
        return out

    def __eq__(self, other):
        if isinstance(other, Transform):
            return self.from_frame == other.from_frame and self.pose == other.pose and self.to_frame == other.to_frame
        return False


def get_root_frames(frame_dict):
    if len(frame_dict) > 0:
        if type(frame_dict.keys()[0]) == str:
            return {k: f for k, f in frame_dict.items() if f.parent not in frame_dict}
        return {k: f for k, f in frame_dict.items() if Path(f.parent) not in frame_dict}
    return {}
