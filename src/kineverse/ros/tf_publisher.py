#!/usr/bin/env python
import rospy
import numpy as np

import kineverse.gradients.common_math  as cm

from jinja2 import Environment, FileSystemLoader, select_autoescape, exceptions

from kineverse.model.paths                  import Path, stopping_set, find_all
from kineverse.model.frames                 import Frame, get_root_frames
from kineverse.model.geometry_model         import RigidBody, EventModel
from kineverse.network.model_client         import ModelClient
from kineverse.time_wrapper                 import Time
from kineverse.type_sets                    import is_symbolic, GM
from kineverse.utils                        import real_quat_from_matrix, res_pkg_path
from kineverse.visualization.ros_visualizer import ROSVisualizer

from multiprocessing import RLock

from tf import TransformBroadcaster

from kineverse_msgs.msg   import ValueMap   as ValueMapMsg
from sensor_msgs.msg import JointState as JointStateMsg

env = Environment(
    loader=FileSystemLoader(res_pkg_path('package://kineverse/resources')),
    autoescape=select_autoescape(['html', 'xml'])
)
urdf_template = env.get_template('urdf_template.jinja')


class ModelTFBroadcaster(object):
    def __init__(self, model_path, model=None):
        self.s_frame_map       = {}
        self.state             = {}
        self.model             = None
        self.model_path        = Path(model_path)
        self.cythonized_matrix = None
        self._state_complete   = False
        self.frame_info        = []
        self.broadcaster       = TransformBroadcaster()
        self.static_frames     = {}
        self.np_poses          = None
        self.lock              = RLock()
        self.visualizer = ROSVisualizer('publisher_debug', 'world')
        self.set_model(model)

    @profile
    def update_state(self, update):
        with self.lock:
            if self.model is not None:

                state = {str(s): v for s, v in update.items()}
                # print('---- Update\n{}'.format('\n'.join(['{}: {}'.format(k, v) for k, v in state.items()])))
                self.state.update(state)
                if not self._state_complete:
                    self._state_complete = len(set(self.cythonized_matrix.str_params).difference(set(self.state.keys()))) == 0
                    if not self._state_complete:
                        return

                self.np_poses = self.cythonized_matrix(**self.state)


    @profile
    def publish_state(self):
        with self.lock:
            if self.np_poses is not None:
                # print('---\n{}'.format('\n'.join(['{}: {}'.format(k, v) for k, v in sorted(self.state.items())])))
                
                indices = set(sum(self.s_frame_map.values(), [])) # set(sum([self.s_frame_map[s] for s in update if s in self.s_frame_map], []))
                now     = Time.now()
                self.visualizer.begin_draw_cycle('debug')
                self.visualizer.draw_poses('debug', cm.eye(4), 0.1, 0.01, [cm.Matrix(self.np_poses[x * 4:x * 4 + 4].tolist()) for x in range(self.np_poses.shape[0]/4)])
                self.visualizer.render('debug')

                published_frames = []
                for x in indices:
                    n, f     = self.frame_info[x]
                    pose     = self.np_poses[x*4: x*4 + 4, :4]
                    position = (pose[0, 3], pose[1, 3], pose[2, 3])
                    quat     = real_quat_from_matrix(pose)
                    self.broadcaster.sendTransform(position, quat, now, n, str(f.parent))
                    # published_frames.append((n, f.parent))

                for n, param in self.static_frames.items():
                    self.broadcaster.sendTransform(param[0], param[1], now, param[2], param[3])
                    # published_frames.append(n)

            # print('---\n{}'.format('\n'.join(['{} -> {}'.format(c, p) for c, p in sorted(published_frames)])))



    def set_model(self, model):
        with self.lock:
            self.model       = model
            self.state       = {}
            self.s_frame_map = {}
            self.static_frames = {}
            self.frame_info  = []
            self.np_poses    = None
            self._state_complete = False
            self.cythonized_matrix = None

            if self.model is not None:
                frames = find_all(self.model_path, model, Frame)

                names, lframes = zip(*frames.items())
                pose_matrix    = cm.vstack(*[f.to_parent if not isinstance(f.to_parent, GM) 
                                                         else f.to_parent.to_sym_matrix() 
                                                         for f in lframes])

                self.frame_info = list(zip([str(n) for n in names], lframes))

                self.cythonized_matrix = cm.speed_up(pose_matrix, cm.free_symbols(pose_matrix))

                self.state = {p: 0.0 for p in self.cythonized_matrix.str_params}
                self._state_complete = True

                self.np_poses = np.vstack([np.eye(4) for x in range(len(self.frame_info))])

                now = Time.now()
                for x, (n, f) in enumerate(self.frame_info):
                    if cm.is_symbolic(f.to_parent):
                        for s in cm.free_symbols(f.to_parent):
                            if s not in self.s_frame_map:
                                self.s_frame_map[s] = []
                            self.s_frame_map[s].append(x)
                    else:
                        to_parent = f.to_parent if not isinstance(f.to_parent, GM) else f.to_parent.to_sym_matrix()
                        position  = (to_parent[0, 3], to_parent[1, 3], to_parent[2, 3])
                        quat      = real_quat_from_matrix(to_parent)
                        self.static_frames[n] = (position, quat, n, str(f.parent))
                        self.broadcaster.sendTransform(position, quat, now, n, str(f.parent))


class ModelTFBroadcaster_URDF(ModelTFBroadcaster):
    def __init__(self, urdf_param, model_path, model=None):
        self.urdf_param  = urdf_param
        
        super(ModelTFBroadcaster_URDF, self).__init__(model_path, model)

    def set_model(self, model):
        super(ModelTFBroadcaster_URDF, self).set_model(model)

        if self.model is not None:
            print('[ModelTFBroadcasterURDF] Model was received.')
            frames = dict(self.frame_info)

            # there can only be one root per published model
            root_frames = get_root_frames(frames)
            if len(root_frames) != 1:
                raise Exception('There should be exactly one root frame in a model. There are: {}\nFrames:\n {}\nRoots:\n {}'.format(len(root_frames), '\n '.join(frames.keys()), '\n '.join(root_frames)))
            root_frame = root_frames.values()[0].parent

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
    def __init__(self, urdf_param, update_topic, model_path, frequency=50, use_js_msg=False):
        super(NetworkedTFBroadcaster, self).__init__(urdf_param, model_path, None)
        self.mc = ModelClient(EventModel)
        self.mc.register_on_model_changed(model_path, self.set_model)

        self.use_js_msg = use_js_msg
        if self.use_js_msg:
            self.sub_state = rospy.Subscriber(update_topic, JointStateMsg, callback=self.cb_state_update, queue_size=1)
        else:
            self.sub_state = rospy.Subscriber(update_topic, ValueMapMsg, callback=self.cb_state_update, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1.0/frequency), self.tick_cb)

    def tick_cb(self, event):
        self.publish_state()

    def cb_state_update(self, msg):
        if self.use_js_msg:
            self.update_state({cm.Symbol(n): v for n, v in zip(msg.name, msg.position)})
        else:
            self.update_state({cm.Symbol(n): v for n, v in zip(msg.symbol, msg.value)})
