import os
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

# Convert from 4x4 matrix composing of rotation matrix and translation vector -> sama dict as a quaternion and translation
# [[rotation (3x3) | translation (1x3)]
#  [ 0     0    0  |       1          ]]
def rt_matrix_to_sama_json(rt_matrix: np.array) -> dict:
    r_quat_xyzw = R.from_matrix(rt_matrix[:3,:3]).as_quat() # quaternion order is xyzw
    sama_dict = {
        'rotation_x' : r_quat_xyzw[0],
        'rotation_y' : r_quat_xyzw[1],
        'rotation_z' : r_quat_xyzw[2],
        'rotation_w' : r_quat_xyzw[3],
        'x' : rt_matrix[0,3],
        'y' : rt_matrix[1,3],
        'z' : rt_matrix[2,3]
    }
    return sama_dict

# Convert from a camera instrinic matrix -> sama dict
# [[f_x, 0,   c_x]
#  [0,   f_y, c_y]
#  [0,   0,   1  ]]
def instrinsic_matrix_to_sama_json(intrinsic_matrix: np.array) -> dict:
    return {
        'f_x' : intrinsic_matrix[0,0],
        'f_y' : intrinsic_matrix[1,1],
        'c_x' : intrinsic_matrix[0,2],
        'c_y' : intrinsic_matrix[1,2],
    }

# Returns the camera calibration parameters in the sama json format
# n.b. Sama format requires the camera->pointcloud pose, not the extrinsics
def camera_calibration_sama_json(rt_matrix: np.array, intrinsic_matrix: np.array) -> dict:
    return rt_matrix_to_sama_json(rt_matrix) | instrinsic_matrix_to_sama_json(intrinsic_matrix)

def extrinsics_to_pose(rt_matrix: np.array) -> np.array:
    return np.linalg.inv(rt_matrix)

def pose_to_extrinsics(rt_matrix: np.array) -> np.array:
    return np.linalg.inv(rt_matrix)
    
def quaternion_to_matrix(quaternion_wxyz) -> np.array: # quaternion order is wxyz
    return o3d.geometry.get_rotation_matrix_from_quaternion(quaternion_wxyz)

# example, combining transofmraitons camera->vehicle, vehicle->world will return camera->world
def frameA_to_frameC(frameA_to_frameB: np.array, frameB_to_frameC: np.array) -> np.array:
    return frameB_to_frameC.dot(frameA_to_frameB)


# array to np.array
# np.array([1,2,3,4]).reshape(2,2)
