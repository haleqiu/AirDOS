import numpy as np
from typing import List

from Pose import Pose

def quaternion_to_SO3(q0, q1, q2, q3):
    return np.array([[1-2*q2*q2-2*q3*q3, 2*q1*q2+2*q0*q3, 2*q1*q3-2*q0*q2],
                     [2*q1*q2-2*q0*q3, 1-2*q1*q1-2*q3*q3, 2*q2*q3+2*q0*q1],
                     [2*q1*q3+2*q0*q2, 2*q2*q3-2*q0*q1, 1-2*q1*q1-2*q2*q2]])

def edn_to_ned(p: Pose) -> Pose:
    T = np.array([[0, 0, 1, 0],
                  [1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 0, 1]])
    # result = T @ p @ T^-1
    result = p.L_transform(T).R_transform(np.linalg.inv(T))
    return result

def align_to_gt_frame(p: Pose) -> Pose:
    T_symmetric      = np.eye(4)
    T_symmetric[0, 0]= -1
    return edn_to_ned(p).L_transform(T_symmetric)

def pose_to_SE3(x, y, z, q0, q1, q2, q3):
    # given pose, transform to SE3 T_(camera frame -> world frame)
    R = quaternion_to_SO3(q0, q1, q2, q3)
    return np.array([[R[0, 0], R[0, 1], R[0, 2], x],
                     [R[1, 0], R[1, 1], R[1, 2], y],
                     [R[2, 0], R[2, 1], R[2, 2], z],
                     [0      , 0      , 0      , 1]])

def sequence_to_SE3(sequence):
    result = []
    for record in range(sequence.shape[0]):
        time, x, y, z, q0, q1, q2, q3 = sequence[record, :]
        result.append(Pose(time, pose_to_SE3(x, y, z, q0, q1, q2, q3)))
    return result

def align_sequence(pose_seq: List[Pose]) -> List[Pose]:
    T_inv = np.linalg.inv(pose_seq[0].SE3)  # T_(world frame -> frame 0)
    d_time = -1 * pose_seq[0].time
    return [pose.L_transform(T_inv, d_time) for pose in pose_seq]

def SE3_to_homogeneous(sequence: List[Pose]) -> List[np.ndarray]:
    initial_position = np.array([[0, 0, 0, 1]]).T
    return np.array([pose.SE3 @ initial_position for pose in sequence])

def SE3_to_heterogeneous(sequence: List[Pose]) -> List[np.ndarray]:
    homogeneous    = SE3_to_homogeneous(sequence)
    hetereogeneous = homogeneous[:, :3, 0] / homogeneous[:, 3:, 0]
    return hetereogeneous
