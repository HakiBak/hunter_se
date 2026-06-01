"""
Utility functions for point cloud processing and transformations.
"""

import math
import numpy as np


def process_point_cloud(points, num_bins=20):
    """
    Process 3D point cloud into distance bins for RL observation.
    """
    velodyne_data = np.ones(num_bins) * 10.0

    if len(points) == 0:
        return velodyne_data
    
    points_list = list(points)
    x = np.array([p[0] for p in points_list])
    y = np.array([p[1] for p in points_list])
    z = np.array([p[2] for p in points_list])


    # Drop non-finite returns (inf/nan from beams that hit nothing) before any math
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    mask = finite & (z > -0.2)  # Filter ground (z≈-0.08 at 0.3m) but keep walls


    x, y, z = x[mask], y[mask], z[mask]

    if len(x) == 0:
        return velodyne_data
    
    mag1 = np.sqrt(x**2 + y**2)
    valid = mag1 > 0.001

    x, y, z, mag1 = x[valid], y[valid],z[valid], mag1[valid]

    if len(x) == 0:
        return velodyne_data
    
    beta = np.arccos(np.clip(x / mag1, -1.0, 1.0)) * np.sign(y)
    dist = np.sqrt(x**2 + y**2 + z**2)
    #=====ADDED PART TO SOLVE THE MINIMUM LIDAR DETECTION RANGE PROBLEM==== 
    # valid_dist = dist > 0.35
    # x, y, z, dist = x[valid_dist], y[valid_dist], z[valid_dist], dist[valid_dist]

    # if len(x) == 0:
    #     return velodyne_data
    #=====ADDED PART TO SOLVE THE MINIMUM LIDAR RANGE==== 

    # valid_dist = dist > 0.35
    # beta = beta[valid_dist]
    # dist = dist[valid_dist]

    # if len(dist) == 0: 
    #     return velodyne_data

    bin_edges = np.linspace(-np.pi / 2 - 0.03, np.pi / 2 + 0.03,
                              num_bins + 1)


    bin_indices = np.digitize(beta, bin_edges) -1 
    bin_indices = np.clip(bin_indices, 0, num_bins - 1)

    for j in range(num_bins):
        bin_mask = bin_indices == j
        if np.any(bin_mask):
            velodyne_data[j] = min(velodyne_data[j], np.min(dist[bin_mask]))

    return velodyne_data


def quaternion_to_euler(w, x, y, z):
    """Convert quaternion to euler angles (roll, pitch, yaw)."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def euler_to_quaternion(roll, pitch, yaw):
    """Convert euler angles to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return w, x, y, z