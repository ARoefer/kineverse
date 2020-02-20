import os
import rospy
import tf
import subprocess

from pprint import pprint

from kineverse.visualization.bpb_visualizer  import ROSBPBVisualizer
from kineverse.gradients.diff_logic          import Symbol, erase_type
from kineverse.model.paths                   import Path
from kineverse.operations.special_kinematics import RoombaJoint
from kineverse.ros.tf_publisher              import ModelTFBroadcaster_URDF
from kineverse.time_wrapper                  import Time

from std_msgs.msg    import Header     as HeaderMsg
from sensor_msgs.msg import JointState as JointStateMsg

from tqdm import tqdm

class TrajectoryVisualizer(object):
    def __init__(self, visualizer=None):
        self.visualizer = visualizer if visualizer is not None else ROSBPBVisualizer('/visuals', '/map')
        self.tf_broadcaster = tf.TransformBroadcaster() 

        self.robot_publishers = {}
        self.js_publishers    = {}
        self.joint_symbols    = {}
        self.base_frames      = {}
        self.devnull          = None

    def add_articulated_object(self, path, art_obj):
        if self.devnull is None:
            self.devnull = open(os.devnull, 'w')

        self.js_publishers[path] = rospy.Publisher('/{}/joint_states'.format(path), JointStateMsg, queue_size=1)
        self.robot_publishers[path] = ModelTFBroadcaster_URDF('/{}/robot_description'.format(path), path, art_obj)

        self.joint_symbols[path] = self.robot_publishers[path].s_frame_map.keys()
        

    def shutdown(self):
        if self.devnull is not None:
            self.devnull.close()


    def visualize(self, traj_dict, hz=50):
        joints      = sum(self.joint_symbols.values(), [])
        traj_len    = len(traj_dict.values()[0])

        traj      = {j: [0.0] * traj_len if str(j) not in traj_dict else traj_dict[str(j)] for j in joints}
        sub_trajs = {n: {Path(erase_type(j))[-1]: traj[j] for j in jns if j in traj} for n, jns in self.joint_symbols.items()}
        loc_trajs = {}

        x     = 0
        step  = 1.0 / hz
        t     = tqdm(total=traj_len, desc='Playing back trajectory...') 
        stamp = Time()
        while not rospy.is_shutdown() and x < traj_len:
            now = Time.now()
            if (now - stamp).to_sec() >= step:
                state = {s: t[x] for s, t in traj.items()}
                for pub in self.robot_publishers.values():
                    pub.update_state(state)
                    pub.publish_state()
                
                stamp = now
                x += 1
                t.update()
        t.close()
