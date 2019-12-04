#!/usr/bin/env python
import rospy

from jinja2 import Environment, FileSystemLoader, select_autoescape

from giskardpy import BACKEND
from kineverse.gradients.gradient_math import spw
from kineverse.model.paths             import Path, stopping_set
from kineverse.model.frames            import Frame
from kineverse.model.geometry_model    import RigidBody, EventModel
from kineverse.network.model_client    import ModelClient
from kineverse.time_wrapper            import Time
from kineverse.type_sets               import is_symbolic
from kineverse.utils                   import real_quat_from_matrix, res_pkg_path

from tf import TransformBroadcaster

from kineverse.msg   import ValueMap   as ValueMapMsg
from sensor_msgs.msg import JointState as JointStateMsg

env = Environment(
    loader=FileSystemLoader(res_pkg_path('package://kineverse/resources')),
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


class ModelTFBroadcaster(object):
    def __init__(self, model_path, model=None):
        self.s_frame_map       = {}
        self.state             = {}
        self.model_path        = Path(model_path)
        self.cythonized_matrix = None
        self._state_complete   = False
        self.frame_info        = []
        self.broadcaster       = TransformBroadcaster()
        self.set_model(model)

    @profile
    def update_state(self, update):
        if self.model is not None:

            self.state.update({str(s): v for s, v in update.items()})
            if not self._state_complete:
                self._state_complete = len(set(self.cythonized_matrix.str_params).difference(set(self.state.keys()))) == 0
                if not self._state_complete:
                    return

            indices = set(sum([self.s_frame_map[s] for s in update if s in self.s_frame_map], []))
            now     = Time.now()

            #print('---\n{}'.format('\n'.join(['{}: {}'.format(k, v) for k, v in sorted(self.state.items())])))

            np_matrix = self.cythonized_matrix(**self.state)
            published_frames = []
            for x in indices:
                n, f     = self.frame_info[x]
                pose     = np_matrix[x*4: x*4 + 4, :4]
                position = (pose[0, 3], pose[1, 3], pose[2, 3])
                quat     = real_quat_from_matrix(pose)
                self.broadcaster.sendTransform(position, quat, now, n, f.parent)
                published_frames.append(n)

            #print('---\n{}'.format('\n'.join(published_frames)))


    def set_model(self, model):
        self.model       = model
        self.state       = {}
        self.s_frame_map = {}
        self.frame_info  = []
        self._state_complete = False

        if self.model is not None:
            frames = find_all_frames(self.model_path, model)

            names, lframes = zip(*frames.items())
            pose_matrix    = lframes[0].to_parent
            for f in lframes[1:]:
                pose_matrix = pose_matrix.col_join(f.to_parent)
            
            self.frame_info = list(zip([str(n) for n in names], lframes))

            self.cythonized_matrix = spw.speed_up(pose_matrix, pose_matrix.free_symbols, backend=BACKEND)

            now = Time.now()
            for x, (n, f) in enumerate(self.frame_info):
                if len(f.to_parent.free_symbols) > 0:
                    for s in f.to_parent.free_symbols:
                        if s not in self.s_frame_map:
                            self.s_frame_map[s] = []
                        self.s_frame_map[s].append(x)
                else:
                    position = (f.to_parent[0, 3], f.to_parent[1, 3], f.to_parent[2, 3])
                    quat     = real_quat_from_matrix(f.to_parent)
                    self.broadcaster.sendTransform(position, quat, now, n, f.parent)


class ModelTFBroadcaster_URDF(ModelTFBroadcaster):
    def __init__(self, urdf_param, model_path, model=None):
        self.urdf_param  = urdf_param
        
        super(ModelTFBroadcaster_URDF, self).__init__(model_path, model)

    def set_model(self, model):
        super(ModelTFBroadcaster_URDF, self).set_model(model)

        if self.model is not None:
            frames = dict(self.frame_info)

            # there can only be one root be published model
            root_frame = list({f.parent for f in frames.values()}.difference(set(frames.values())))[0]

            fs  = {}
            rbs = {}
            for p, f in frames.items():
                if isinstance(f, RigidBody) and f.geometry is not None:
                    rbs[p] = f
                else:
                    fs[p] = f

            model_urdf = urdf_template.render(root=root_frame, frames=fs, rbs=rbs)
            rospy.set_param(self.urdf_param, model_urdf)

            with open('temp_{}.urdf'.format(str(self.model_path.to_symbol())), 'w') as temp:
                temp.write(model_urdf)
        else:
            rospy.set_param(self.urdf_param, '')


class NetworkedTFBroadcaster(ModelTFBroadcaster_URDF):
    def __init__(self, urdf_param, update_topic, model_path, use_js_msg=False):
        super(NetworkedTFBroadcaster, self).__init__(urdf_param, model_path, None)
        self.mc = ModelClient(EventModel)
        self.mc.register_on_model_changed(model_path, self.set_model)

        self.use_js_msg = use_js_msg
        if self.use_js_msg:
            self.sub_state = rospy.Subscriber(update_topic, JointStateMsg, callback=self.cb_state_update, queue_size=1)
        else:
            self.sub_state = rospy.Subscriber(update_topic, ValueMapMsg, callback=self.cb_state_update, queue_size=1)

    def cb_state_update(self, msg):
        if self.use_js_msg:
            self.update_state({Symbol(n): v for n, v in zip(msg.name, msg.position)})
        else:    
            self.update_state({Symbol(n): v for n, v in zip(msg.symbol, msg.value)})