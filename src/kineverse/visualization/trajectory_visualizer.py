import os
import rospy
import tf
import subprocess

from pprint import pprint

from kineverse.visualization.bpb_visualizer  import ROSBPBVisualizer
from kineverse.gradients.diff_logic          import Symbol, erase_type
from kineverse.model.paths                   import Path
from kineverse.operations.special_kinematics import RoombaJoint
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

    def add_articulated_object(self, urdf, art_obj):
        if self.devnull is None:
            self.devnull = open(os.devnull, 'w')

        self.js_publishers[art_obj.name] = rospy.Publisher('/{}/joint_states'.format(art_obj.name), JointStateMsg, queue_size=1)
        self.robot_publishers[art_obj.name] = subprocess.Popen([
            '/opt/ros/{}/lib/robot_state_publisher/robot_state_publisher'.format(os.environ['ROS_DISTRO']),
            '__name:={}_state_publisher'.format(art_obj.name),
            'robot_description:=/{}/robot_description'.format(art_obj.name),
            '_tf_prefix:={}'.format(art_obj.name),
            'joint_states:=/{}/joint_states'.format(art_obj.name)], stdout=self.devnull)
        rospy.set_param('/{}/robot_description'.format(art_obj.name), urdf.to_xml_string())

        self.joint_symbols[art_obj.name] = [j.position for j in art_obj.joints.values() if hasattr(j, 'position') and type(j.position) is Symbol]
        for j in art_obj.joints.values():
            if isinstance(j, RoombaJoint):
                self.joint_symbols[art_obj.name].append(j.x_pos)
                self.joint_symbols[art_obj.name].append(j.y_pos)
                self.joint_symbols[art_obj.name].append(j.z_pos)
                self.joint_symbols[art_obj.name].append(j.a_pos)

        self.base_frames[art_obj.name] = '{}/{}'.format(art_obj.name, urdf.get_root())

    def shutdown(self):
        for n, p in tqdm(self.robot_publishers.items(), desc='Ending {} robot publishers.'.format(len(self.robot_publishers))):
            self.js_publishers[n].unregister()
            p.terminate()
            p.wait()

        if self.devnull is not None:
            self.devnull.close()


    def visualize(self, traj_dict, hz=50):
        joints      = sum(self.joint_symbols.values(), [])
        traj_len    = len(traj_dict.values()[0])
        loc_symbols = [j for j in joints if Path(j)[-1][:13] == 'localization_']

        traj      = {j: [0.0] * traj_len if str(j) not in traj_dict else traj_dict[str(j)] for j in joints}
        sub_trajs = {n: {Path(erase_type(j))[-1]: traj[j] for j in jns if j in traj} for n, jns in self.joint_symbols.items()}
        loc_trajs = {}


        for n, d in sub_trajs.items():
            loc_x = d['localization_x'] if 'localization_x' in d else [0.0] * traj_len
            loc_y = d['localization_y'] if 'localization_y' in d else [0.0] * traj_len
            loc_z = d['localization_z'] if 'localization_z' in d else [0.0] * traj_len
            loc_a = d['localization_a'] if 'localization_a' in d else [0.0] * traj_len
            loc_trajs[n] = zip(loc_x, loc_y, loc_z, loc_a)

        msgs = {n: JointStateMsg(name=d.keys()) for n, d in sub_trajs.items()}

        x    = 0
        step = 1.0 / hz
        t    = tqdm(total=traj_len, desc='Playing back trajectory...') 
        while not rospy.is_shutdown() and x < traj_len:
            now = Time.now()
            if (now - msgs.values()[0].header.stamp).to_sec() >= step:
                for n, msg in msgs.items():
                    msg.header.stamp = now
                    msg.position     = [sub_trajs[n][j][x] for j in msg.name]
                    self.js_publishers[n].publish(msg)
                    if n in loc_trajs:
                        self.tf_broadcaster.sendTransform(loc_trajs[n][x][:3],
                                        tf.transformations.quaternion_from_euler(0,0, loc_trajs[n][x][3]),
                                        now, self.base_frames[n], self.visualizer.base_frame)
                
                x += 1
                t.update()
        t.close()



