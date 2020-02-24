import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

imu_pose = np.loadtxt('imu_pose.txt')
imu_pose_noise = np.loadtxt('imu_pose_noise.txt')

fig = plt.figure()
ax = fig.gca(projection='3d')

ax.plot(imu_pose[:,0], imu_pose[:,1], imu_pose[:,2], label='imu_pose')
ax.plot(imu_pose_noise[:,0], imu_pose_noise[:,1], imu_pose_noise[:,2], label='imu_pose_noise')

ax.legend()
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()
