#!/usr/bin/env python

import rospy

from kineverse.gradients.gradient_math      import *
from kineverse.visualization.ros_visualizer import ROSVisualizer 
from kineverse.time_wrapper                 import Time
from kineverse.utils                        import real_quat_from_matrix


def get_rot_vector(frame):
    qx, qy, qz, _ = real_quat_from_matrix(frame)

    axis   = vector3(qx, qy, qz)
    sin_a2 = norm(axis)
    a      = asin(sin_a2) * 2
    return axis * (a / sin_a2)


if __name__ == '__main__':
    rospy.init_node('axis_angle_vis')

    vis = ROSVisualizer('axis_vis', 'map')

    az, ay = [Position(x) for x in 'ax az'.split(' ')]
    frame_rpy = frame3_rpy(0, ay, az, point3(0,0,0))

    state = {ay: 0, az: 0}

    points = [point3(0,0,0) + get_rot_vector(frame_rpy.subs({ay: sin(v), az: cos(v)})) for v in [(3.14512 / 25) * x for x in range(51)]]

    vis.begin_draw_cycle('points')
    vis.draw_strip('points', spw.eye(4), 0.03, points)
    vis.render('points')

    rospy.sleep(1)

    timer = Time()
    while not rospy.is_shutdown():
        now = Time.now()

        if (now - timer).to_sec() >= 0.02:
            state[ay] = sin(now.to_sec())
            state[az] = cos(now.to_sec())

            frame = frame_rpy.subs(state)
            axis  = get_rot_vector(frame)

            vis.begin_draw_cycle('visuals')
            vis.draw_poses('visuals', spw.eye(4), 0.4, 0.02, [frame])
            vis.draw_vector('visuals', pos_of(frame), axis)
            vis.render('visuals')
