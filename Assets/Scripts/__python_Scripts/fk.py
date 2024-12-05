import numpy as np

# Helper function to calculate DH transformation matrix
def dh_matrix(a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

# DH Parameters (adjusted based on extracted URDF data)
# Link lengths (a), link twists (alpha), link offsets (d), joint angles (theta)
dh_parameters = [
    (0.103, -np.pi / 2, 0.063, 0),  # joint_1
    (0.080, 0, 0, 0),              # joint_2
    (0.210, -np.pi / 2, 0, 0),     # joint_3
    (0.0415, np.pi / 2, 0, 0),     # joint_4
    (0, -np.pi / 2, 0, 0),         # joint_5
    (0, 0, 0.072, 0)               # joint_6
]

# FK computation
def forward_kinematics(joint_angles):
    T = np.eye(4)
    for i, (a, alpha, d, theta) in enumerate(dh_parameters):
        T = np.dot(T, dh_matrix(a, alpha, d, joint_angles[i]))
    return T

# Example joint angles within limits (in radians)
joint_angles = [
    np.radians(0),      # joint_1
    np.radians(-30),    # joint_2
    np.radians(45),     # joint_3
    np.radians(90),     # joint_4
    np.radians(-45),    # joint_5
    np.radians(30)      # joint_6
]

# Compute FK
end_effector_pose = forward_kinematics(joint_angles)

# Print result
print("End Effector Pose (4x4 Transformation Matrix):")
print(end_effector_pose)
