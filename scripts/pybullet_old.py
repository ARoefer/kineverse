import numpy as np
import pybullet as pb

from time import time

from kineverse.utils import res_pkg_path

if __name__ == '__main__':
    
    client = pb.connect(pb.DIRECT)

    pb.setGravity(0,0,-10)

    kitchen = pb.loadURDF(res_pkg_path('package://iai_kitchen/urdf_obj/iai_kitchen_python.urdf'), useFixedBase=False, basePosition=(0,0,0), baseOrientation=(0,0,0,1))

    suzanne = pb.loadURDF(res_pkg_path('package://kineverse/urdf/suzanne.urdf'), useFixedBase=False, basePosition=(0,0,0), baseOrientation=(0,0,0,1))

    print('kitchen Id: {} Suzanne Id: {}'.format(kitchen, suzanne))

    # pb.stepSimulation()

    times = []
    for x in range(1000):
        start = time()
        result = pb.getClosestPoints(suzanne, kitchen, 10.0, 0, -1)
        end = time()
        times.append(end - start)
        print(pb.getBasePositionAndOrientation(suzanne))
        print('Iteration {}/1000'.format(x + 1))

    np_times_ms = np.array(times) * 1000
    print('Mean query time: {} ms\n SD: {}\n Min: {}\n Max: {}'.format(np_times_ms.mean(), np_times_ms.std(), np_times_ms.min(), np_times_ms.max()))

    pb.disconnect()
