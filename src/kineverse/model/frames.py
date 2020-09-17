"""
The frames module provides a basic implementation of the concept of 3d frames and transformations.
"""

import kineverse.gradients.common_math as cm

from kineverse.model.paths             import Path
from kineverse.json_wrapper            import JSONSerializable

class Frame(JSONSerializable):
    """Frame that stores the path to its parent, its world transform and its local transform.
    NOTE: The path is stored as str, this is a requirement from the way operations work.
    """
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
            return self.parent == other.parent       and \
                   cm.eq_expr(self.pose, other.pose) and \
                   cm.eq_expr(self.to_parent, other.to_parent)
        return False


class Transform(JSONSerializable):
    """A relative transform between two frames."""
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
            return self.from_frame == other.from_frame and \
                   cm.eq_expr(self.pose, other.pose)   and \
                   cm.eq_expr(self.to_frame, other.to_frame)
        return False


def get_root_frames(frame_dict):
    """Given a dictionary of frames, identifies the roots of the kinematic chains."""
    if len(frame_dict) > 0:
        if type(frame_dict.keys()[0]) == str:
            return {k: f for k, f in frame_dict.items() if str(f.parent) not in frame_dict}
        return {k: f for k, f in frame_dict.items() if f.parent not in frame_dict}
    return {}
