from urdfpy import URDF
import pandas as pd

# Load the URDF file
urdf_path = "niryo_one.urdf"  # Replace with the path to your URDF file
robot = URDF.load(urdf_path)

# Extract DH-relevant parameters
dh_parameters = []

for joint in robot.joints:
    if joint.joint_type == 'revolute':
        name = joint.name
        axis = joint.axis
        origin_xyz = joint.origin[0] if joint.origin else (0, 0, 0)
        origin_rpy = joint.origin[1] if joint.origin else (0, 0, 0)
        
        dh_parameters.append({
            "Joint": name,
            "Axis": axis,
            "Origin XYZ": origin_xyz,
            "Origin RPY": origin_rpy
        })

# Convert to DataFrame for easier visualization
dh_df = pd.DataFrame(dh_parameters)

# Save or print the DataFrame
print(dh_df)

# Optionally, save to CSV
dh_df.to_csv("dh_parameters.csv", index=False)
