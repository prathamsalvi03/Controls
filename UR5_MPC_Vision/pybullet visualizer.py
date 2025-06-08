import pybullet as p
import pybullet_data
import time
import numpy as np

# === Connect to PyBullet ===
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# === Environment setup ===
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)  # Step simulation manually

# Load ground plane and robot
p.loadURDF("plane.urdf")
urdf_path = "/home/pratham/Pratham/example-robot-data-master/robots/ur_description/urdf/ur5_robot.urdf"
startPos = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robot_id = p.loadURDF(urdf_path, startPos, startOrientation, useFixedBase=True)

# === Identify joints ===
num_joints = p.getNumJoints(robot_id)
print(f"Number of joints: {num_joints}")

revolute_joint_indices = []
for i in range(num_joints):
    joint_info = p.getJointInfo(robot_id, i)
    joint_name = joint_info[1].decode('utf-8')
    joint_type = joint_info[2]
    print(f"Joint {i}: {joint_name}, Type: {joint_type}")
    if joint_type == p.JOINT_REVOLUTE:
        revolute_joint_indices.append(i)

print(f"Revolute joints: {revolute_joint_indices}")

# === Load joint angles ===
try:
    smoothed_angles = np.load("arm/smoothed_angles.npy")
    print(f"Loaded smoothed_angles: {smoothed_angles}")
except FileNotFoundError:
    print("❌ Error: smoothed_angles.npy not found.")
    p.disconnect()
    exit(1)

# === Apply smoothed joint angles ===
for i, joint_index in enumerate(revolute_joint_indices):
    if i < len(smoothed_angles):
        p.setJointMotorControl2(
            robot_id,
            joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=smoothed_angles[i],
            force=100
        )

# === Set camera view ===
p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
    cameraYaw=0,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0.5]
)

# === Step simulation for a few seconds to observe motion ===
for _ in range(240):  # ~1 second of simulation at 240 Hz
    p.stepSimulation()
    time.sleep(1/240.)

# === Clean exit ===
p.disconnect()
