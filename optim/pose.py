import numpy as np
import torch
import pytorch3d.transforms


def transform_to_tensor(transformation_matrix, device=None):

    gpu_id = -1
    if isinstance(transformation_matrix, np.ndarray):
        transformation_matrix = torch.from_numpy(transformation_matrix)
    elif not isinstance(transformation_matrix, torch.Tensor):
        raise ValueError((f"Invalid argument of type {type(transformation_matrix).__name__}"
                          "passed to transform_to_tensor (Expected numpy array or pytorch tensor)"))

    R = transformation_matrix[:3, :3]
    T = transformation_matrix[:3, 3]

    rot = pytorch3d.transforms.matrix_to_axis_angle(R)

    tensor = torch.cat([T, rot]).float()
    if device is not None:
        tensor = tensor.to(device)
    elif gpu_id != -1:
        tensor = tensor.to(gpu_id)
    return tensor


## Converts a tensor produced by transform_to_tensor to a transformation matrix
# Inspired by a similar NICE-SLAM function.
# @param transformation_tensors: se(3) twist vectors
# @returns a 4x4 homogenous transformation matrix
def tensor_to_transform(transformation_tensors):

    N = len(transformation_tensors.shape)
    if N == 1:
        transformation_tensors = torch.unsqueeze(transformation_tensors, 0)
    Ts, rots = transformation_tensors[:, :3], transformation_tensors[:, 3:]
    rotation_matrices = pytorch3d.transforms.axis_angle_to_matrix(rots)
    RT = torch.cat([rotation_matrices, Ts[:, :, None]], 2)
    if N == 1:
        RT = RT[0]

    H_row = torch.zeros_like(RT[0])
    H_row[3] = 1
    RT = torch.vstack((RT, H_row))
    return RT

class Pose:
    """Simplified optimizable pose class.
    Poses are represented as [x, y, z, axis-angle].
    """

    def __init__(self, pose_tensor=None, fixed=False):
        if pose_tensor is None:
            pose_tensor = torch.tensor([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Default at origin with no rotation
        self.pose_tensor = pose_tensor.clone().detach().requires_grad_(not fixed)

    def set_fixed(self, fixed=True):
        self.pose_tensor.requires_grad_(not fixed)

    def get_translation(self):
        return self.pose_tensor[:3]

    def get_rotation_matrix(self):
        aa = self.pose_tensor[3:]
        return pytorch3d.transforms.axis_angle_to_matrix(aa)

    def get_transformation_matrix(self):
        return tensor_to_transform(self.pose_tensor)

    def inverse(self):
        inv_rot = self.get_rotation_matrix().T
        inv_trans = -inv_rot @ self.get_translation()
        new_transform = torch.eye(4).to(inv_rot)
        new_transform[:3, :3] = inv_rot
        new_transform[:3, 3] = inv_trans
        return Pose(transform_to_tensor(new_transform))

    def __mul__(self, other: "Pose"):
        new_T = self.get_transformation_matrix() @ other.get_transformation_matrix()
        return Pose(transform_to_tensor(new_T))

    def clone(self, fixed=None):
        return Pose(self.pose_tensor.clone(), fixed=fixed if fixed is not None else not self.pose_tensor.requires_grad)

    def to(self, device):
        self.pose_tensor = self.pose_tensor.to(device)
        return self

    def detach(self):
        return Pose(self.pose_tensor.detach(), fixed=True)

    def __repr__(self):
        return f"Pose(translation={self.get_translation().tolist()}, quaternion={self.pose_tensor[3:].tolist()})"

