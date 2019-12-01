#!/usr/bin/env python
import rospy

from jinja2 import Environment, PackageLoader, select_autoescape

from kineverse.model.paths          import Path, stopping_set
from kineverse.model.frames         import Frame
from kineverse.model.geometry_model import RigidBody
from kineverse.network.model_client import ModelClient
from kineverse.time_wrapper         import Time
from kineverse.type_sets            import is_symbolic
from kineverse.utils                import real_quat_from_matrix


from tf import TransformBroadcaster

from sensor_msgs.msg import JointState as JointStateMsg

env = Environment(
    loader=PackageLoader('kineverse', 'resources'),
    autoescape=select_autoescape(['html', 'xml'])
)
urdf_template = env.get_template('urdf_template.jinja')

def find_all_frames(path, model):
    if isinstance(model, Frame):
        return {path: model}

    out = {}
    t   = type(model)
    if t not in stopping_set:
        if t is dict:
            for k, v in model.items():
                if type(k) is str:
                    out.update(find_all_frames(path + (k,), v))
        elif t is list:
            for x, d in enumerate(model):
                out.update(find_all_frames(path + (x,), d))
        else:
            for a in [a for a in dir(model) if a[0] != '_' and not callable(getattr(model, a))]:
                out.update(find_all_frames(path + (a,), getattr(model, a)))
    return out


class ModelVisualizer(object):
    def __init__(self, urdf_param, update_topic, model_path):
        self.model = None
        self.urdf_param  = urdf_param
        self.s_frame_map = {}
        self.state = {}
        self.broadcaster = TransformBroadcaster()
        self.mc = ModelClient()

        self.mc.register_on_model_changed(model_path, self.cb_model_changed)

        self.sub_state = rospy.Subscriber(update_topic, JointStateMsg, callback=self.cb_state_update, queue_size=0)
        print('TF-Model publisher for {} is ready.'.format(model_path))


    def cb_state_update(self, msg):
        if model is not None:
            msg.name = [Symbol(n) for n in msg.name]
            frames = set(sum([self.s_frame_map[s] for s in msg.name if s in self.s_frame_map], []))

            self.state.update(dict(zip(msg.name, msg.position)))

            now = Time.now()
            for n, f in frames:
                pose = f.pose.subs(self.state)
                if not is_symbolic(pose):
                    self.broadcaster.sendTransform((pose[0, 3], pose[1, 3], pose[2, 3]), real_quat_from_matrix(pose), now, n, f.parent)


    def cb_model_changed(self, model):
        self.model       = model
        self.state       = {}
        self.s_frame_map = {}

        if self.model is not None:
            frames = find_all_frames(Path(''), model)

            # there can only be one root be published model
            root_frame = list({f.parent for f in frames.values()}.difference(set(frames.values())))[0]

            fs  = {}
            rbs = {}
            for p, f in frames.items():
                for s in f.pose.free_symbols:
                    if s not in self.s_frame_map:
                        self.s_frame_map[s] = []
                    self.s_frame_map[s].append((str(p), f))

                if isinstance(f, RigidBody):
                    rbs[n] = f
                else:
                    fs[n] = f

            model_urdf = urdf_template.render(root=root_frame, frames=fs, rbs=rbs)
            rospy.set_param(self.urdf_param, model_urdf)

            print('New model urdf: {}'.format(model_urdf))


if __name__ == '__main__':
    rospy.init_node()

    js_topic   = rospy.get_param('joint_states', 'joint_states')
    desc_param = rospy.get_param('robot_description', 'robot_description')
    model      = rospy.get_param('model_path')

    while not rospy.is_shutdown():
        rospy.sleep(1000)
