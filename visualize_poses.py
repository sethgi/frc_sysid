import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

def euler_to_rot_matrix(roll, pitch, yaw):
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    return R_z @ R_y @ R_x

def visualize_poses(poses):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([-10, 10])
    ax.set_ylim([-10, 10])
    ax.set_zlim([-10, 10])

    trajectory = np.array([[pose[0], pose[1], pose[2]] for pose in poses])
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], 'gray', linestyle='--')

    frame = {'quiver': None}

    def update(num):
        ax.collections.clear()
        pose = poses[num]
        x, y, z, roll, pitch, yaw = pose
        R = euler_to_rot_matrix(roll, pitch, yaw)

        # Draw the coordinate frame
        origin = np.array([x, y, z])
        axis_length = 1.0
        axes = np.eye(3) * axis_length
        transformed_axes = R @ axes

        for i, color in enumerate(['r', 'g', 'b']):
            ax.quiver(origin[0], origin[1], origin[2],
                      transformed_axes[0, i], transformed_axes[1, i], transformed_axes[2, i],
                      color=color)

    ani = FuncAnimation(fig, update, frames=len(poses), interval=200)
    plt.show()

# Example poses: [x, y, z, roll, pitch, yaw] in radians
poses = [
    [0, 0, 0, 0, 0, 0],
    [1, 1, 1, np.pi/6, np.pi/6, np.pi/6],
    [2, 2, 0, np.pi/4, np.pi/4, np.pi/4],
    [3, 3, 1, np.pi/3, np.pi/3, np.pi/3],
    [4, 4, 2, np.pi/2, np.pi/2, np.pi/2]
]

visualize_poses(poses)
