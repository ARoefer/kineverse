#!/usr/bin/env python
import numpy as np
import rospy
import math
import betterpybullet as pb


from kineverse.time_wrapper import Time
from kineverse.utils import res_pkg_path
from kineverse.visualization.bpb_visualizer import ROSBPBVisualizer

COLORS = [(1,1,1,1), (1,0,0,1), (0,1,0,1), (0,0,1,1), (1,1,0,1), (1,0,1,1), (0,1,1,1)]

def create_object(shape, transform=pb.Transform.identity()):
    out = pb.CollisionObject()
    out.set_collision_shape(shape)
    out.collision_flags = pb.CollisionObject.KinematicObject
    out.transform = transform
    return out

if __name__ == '__main__':
    rospy.init_node('kineverse_bullet_test')

    vis = ROSBPBVisualizer('bullet_test', 'map')

    kw = pb.KineverseWorld()
    kw.force_update_all_aabbs = True

    suzanne_obj = pb.load_convex_shape(res_pkg_path("package://kineverse/meshes/suzanne.obj"))
    suzanne_stl = pb.load_convex_shape(res_pkg_path("package://kineverse/meshes/suzanne.stl"))

    cube_shape = pb.BoxShape(pb.Vector3(0.5,0.5,0.5))
    cube_body  = create_object(cube_shape, pb.Transform(1, 0, 0))
    cube2_body = create_object(cube_shape, pb.Transform(-1, 0, 0))
    #obj_body   = create_object(suzanne_obj, pb.Transform(0, 1, 0)) 
    #stl_body   = create_object(suzanne_stl, pb.Transform(0,-1, 0))


    kw.add_collision_object(cube_body)
    kw.add_collision_object(cube2_body)
    #kw.add_collision_object(obj_body)
    #kw.add_collision_object(stl_body)
    
    cube_body.activate(True)
    cube2_body.activate(True)
    #obj_body.activate(True)
    #stl_body.activate(True)
    
    #print(cube_body.isActive, cube2_body.isActive, obj_body.isActive, stl_body.isActive)
    #print(cube_body.activation_state, cube2_body.activation_state, obj_body.activation_state, stl_body.activation_state)

    last_time = Time.now()
    while not rospy.is_shutdown():
        now = Time.now()
        dt  = now - last_time
        if dt.to_sec() >= 0.02:
            last_time = now
            y = math.sin(now.to_sec())
            vis.begin_draw_cycle('objects', 'contacts')
            # pose_array = np.array([[1,0,0,-y - 1], 
            #                        [0,1,0,0], 
            #                        [0,0,1,0], 
            #                        [0,0,0,1],
            #                        [1,0,0,y + 1], 
            #                        [0,1,0,0], 
            #                        [0,0,1,0], 
            #                        [0,0,0,1],
            #                        [1,0,0,0], 
            #                        [0,1,0,y + 1], 
            #                        [0,0,1,0], 
            #                        [0,0,0,1],
            #                        [1,0,0,0], 
            #                        [0,1,0,-y - 1], 
            #                        [0,0,1,0], 
            #                        [0,0,0,1]])

            cube2_body.transform = pb.Transform(pb.Quaternion(pb.Vector3(1,1,0).normalized(), y), pb.Vector3(-y - 1, 0, 0))
            cube_body.transform = pb.Transform(pb.Quaternion(pb.Vector3(0,1,0), y), pb.Vector3(y + 1, 0, 0))
            # obj_body.transform = pb.Transform(0,  y + 1, 0)
            # stl_body.transform = pb.Transform(0, -y - 1, 0)

            #kw.batch_set_transforms([obj_body, cube2_body, cube_body, stl_body], pose_array)

            kw.update_aabbs()
            vis.draw_world('objects', kw)
            kw.perform_discrete_collision_detection()

            #contacts = kw.get_contacts()
            closest = kw.get_closest_batch({cube_body: 2})#, stl_body: 2})
            #print('Number of contacts: {}'.format(len(closest)))
            for contacts in closest.values():
                for i, cp in enumerate(contacts):
                    lines = sum([[cp.obj_a.transform * p.point_a, cp.obj_b.transform * p.point_b] for p in cp.points], [])
                    #print(lines)
                    c = COLORS[i % len(COLORS)]
                    vis.draw_lines('contacts', pb.Transform.identity(), 0.05, lines, r=c[0], g=c[1], b=c[2])

            vis.render()

