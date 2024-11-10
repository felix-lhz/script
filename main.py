import time
import numpy as np
# 创建机械臂对象
import rospy
from dofbot_real import RealEnv

if __name__ == '__main__':
    env = RealEnv()
    env.reset()

    points = [
        np.asarray([90., 90., 90., 90., 90.]),
        np.asarray([136.0, 50.0, 53.0, 1.0, 86.0]),
        np.asarray([136.0, 50.0, 53.0, 1.0, 86.0]),
        np.asarray([136.0, 70.0, 53.0, 1.0, 86.0]),
        np.asarray([180 - 138.0, 70.0, 53.0, 1.0, 86.0]),
        np.asarray([180 - 138.0, 55.0, 53.0, 1.0, 86.0]),
        # np.asarray([136.0, 90.0, 53.0, 1.0, 86.0]),
        # np.asarray([90.0, 50.0, 53.0, 1.0, 86.0]),
        np.asarray([180 - 138.0, 65.0, 53.0, 1.0, 86.0]),
    ]

    split = [30, 5, 30, 40, 10, 10]
    gripper = [-1, -1, 140., -1., -1, -1, 10.]


    def linear_interpolation(src, tat, n=10):
        path = np.linspace(src, tat, num=n)
        return path


    for i in range(len(points) - 1):
        if gripper[i + 1] < 0.:
            path = linear_interpolation(points[i], points[i + 1], n=split[i])
        else:
            path = linear_interpolation(env.get_state()[-1], gripper[i + 1], n=split[i])
        for p in path:
            if gripper[i + 1] < 0.:
                env.step(p)
            else:
                env.step(gripper=p)
