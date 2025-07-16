from franky import Robot, Gripper, JointMotion
import os
import yaml
from python_utils.utils import get_workspace_root

# Initialize the robot
ip = "10.0.2.103"
rbt = Robot(ip)

# Fetch the real joint positions from the robot
calibration_joint_pos = rbt.current_joint_positions
initial_match_joint_pos = rbt.current_joint_positions

# Path to the configuration file
config_file_name = 'franka_example.yaml'
config_path = os.path.join(get_workspace_root(), f"src/factr_teleop/factr_teleop/configs/{config_file_name}")

# Load the configuration file
with open(config_path, 'r') as config_file:
    print(f"Loading configuration from {config_path}")
    config = yaml.safe_load(config_file)

# Update the YAML configuration with real joint positions
config["arm_teleop"]["initialization"]["calibration_joint_pos"] = calibration_joint_pos.tolist()
config["arm_teleop"]["initialization"]["initial_match_joint_pos"] = initial_match_joint_pos.tolist()

# Optionally, save the updated config back to the YAML file
with open(config_path, 'w') as config_file:
    yaml.safe_dump(config, config_file)

print("Configuration updated with real joint positions.")
