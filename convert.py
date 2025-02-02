import numpy as np
from scipy.spatial.transform import Rotation

def to_transform(vec):
    t = vec[:3]
    r = Rotation.from_euler('xyz', vec[3:], True).as_matrix()
    result = np.eye(4)
    result[:3, :3] = r
    result[:3, 3] = t
    return result

targetpose_cameraspace = np.array([-0.036102277065816264, 0.6584905171821219, 2.647370055689336, -19.988610582711967, 10.24124652287433, -1.01978463490838])
botpose_targetspace    = np.array([0.602226373596043, 0.5010764312150219, -3.0863263898736437, -9.879155144091616, -9.970459945048638, -2.5628820991662806])
botpose_worldspace     = np.array([-8.202727188904499, -0.6023222786740555, 0.0, 0.0, 0.0, 9.52047679902328])
robot_to_LL = np.array([0.381, 0, 0.305, 0, 30, 0])

T_world_target = np.array([-1,-1.2246467991473532e-16,0,-5.116399999999999,
                          1.2246467991473532e-16,-1,0,-0.00009999999999976694,
                          0,0,1,0.308102,
                          0,0,0,1]).reshape(4,4)

R_world_target = np.array([[0,0,-1], [1,0,0], [0,-1,0]])
Tr_world_target = np.eye(4)
Tr_world_target[:3, :3] = R_world_target

T_world_robot = to_transform(botpose_worldspace)

T_cam_target = to_transform(targetpose_cameraspace)

T_target_bot = to_transform(botpose_targetspace)


T_robot_cam = to_transform(robot_to_LL)

