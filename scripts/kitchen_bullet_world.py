import rospy

from iai_bullet_sim.realtime_simulator_node import FixedTickSimulator
from iai_bullet_sim.ros_plugins             import TrajectoryPositionController, ResetTrajectoryPositionController
from iai_bullet_sim.full_state_node         import FullStatePublishingNode
from iai_bullet_sim.srv                     import AddURDFRequest, AddRigidObjectRequest
from fetch_giskard.simulator_plugins        import FetchDriver

if __name__ == '__main__':
    rospy.init_node('kineverse_bullet_sim')

    node = FixedTickSimulator(FullStatePublishingNode)
    node.init(mode='direct')

    req = AddRigidObjectRequest()
    req.geom_type = 'box'
    req.extents.x = req.extents.y = 10
    req.extents.z = 1
    req.pose.position.z = -0.5
    req.pose.orientation.w = 1
    req.name = 'floor'

    node.srv_add_rigid_body(req) # This is dumb!

    req = AddURDFRequest()
    req.urdf_path  = 'package://iai_kitchen/urdf_obj/IAI_kitchen.urdf'
    req.name       = 'iai_kitchen'
    req.fixed_base = True
    req.pose.orientation.w = 1

    node.srv_add_urdf(req) # This is reeeeeeally stupid!

    req.urdf_path  = 'package://fetch_description/robots/fetch.urdf'
    req.name       = 'fetch'
    req.fixed_base = False
    req.pose.orientation.w = 1

    node.srv_add_urdf(req) # Still reeeeeeally stupid!

    sim     = node.sim
    kitchen = sim.get_body('iai_kitchen')
    fetch   = sim.get_body('fetch')
    fetch.joint_driver = FetchDriver(1, 0.6)

    sim.register_plugin(TrajectoryPositionController(fetch, 'fetch'))

    node.run()

    while not rospy.is_shutdown():
        pass

    node.kill()