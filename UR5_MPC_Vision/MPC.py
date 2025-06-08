# import numpy as np
# import cvxpy as cp
# import matplotlib.pyplot as plt
# from ik_solver import solve_ik_ur5  # Replace with your IK solver function

# # System Parameters for MPC

# A = np.array([[0, 1], [0, 0]])  # Continuous-time state matrix
# B = np.array([[0], [1]])        # Continuous-time input matrix
# T = 0.1                         # Sampling time

# # Discretization using Forward Euler Method
# A_d = np.eye(2) + T * A
# B_d = T * B

# # MPC Parameters
# N = 10                          # Prediction horizon
# Q = np.eye(2)                   # State cost matrix
# R = np.eye(1) * 0.1             # Control input cost matrix
# x0 = np.array([[0], [0]])       # Initial state (position=0, velocity=0)
# xd = np.array([[1], [0]])       # Desired state (target position=1, velocity=0)

# # Matrices for state and input trajectory
# x = cp.Variable((2, N + 1))  # State trajectory
# u = cp.Variable((1, N))      # Input trajectory

# # Cost function and constraints
# cost = 0
# constraints = [x[:, 0] == x0.flatten()]  # Initial state constraint

# for k in range(N):
#     cost += cp.quad_form(x[:, k] - xd.flatten(), Q) + cp.quad_form(u[:, k], R)
#     constraints += [x[:, k + 1] == A_d @ x[:, k] + B_d @ u[:, k]]
#     constraints += [cp.norm(u[:, k], 'inf') <= 10]  # Input constraint (torque limits)

# # Final state cost
# cost += cp.quad_form(x[:, N] - xd.flatten(), Q)

# # Solve the optimization problem
# problem = cp.Problem(cp.Minimize(cost), constraints)
# problem.solve()

# # Results from MPC
# trajectory = x.value[0, :]  # Optimal position trajectory
# control_inputs = u.value    # Optimal control inputs (torques)

# # Initialize joint angles (assuming 2D planar arm for simplicity)
# joint_angles = []

# urdf_path = "/home/pratham/Pratham/example-robot-data-master/robots/ur_description/urdf/ur5_robot.urdf"

# # Loop through MPC positions and solve IK
# print("Solving Inverse Kinematics for MPC trajectory...")
# for position in trajectory:
#     desired_position = np.array([position, 0, 0])  # Add z=0 for a 3D end-effector position
#     angles = solve_ik_ur5(urdf_path, joint_id=6, target_position=desired_position)
#     if angles is not None:
#         joint_angles.append(angles.flatten())
#     else:
#         print(f"Failed to solve IK for position: {desired_position}")
#         joint_angles.append(np.nan)
# # Convert joint angles to a NumPy array for easier plotting
# joint_angles = np.array(joint_angles)

# # Plot the results
# time = np.linspace(0, N * T, N + 1)

# plt.figure(figsize=(12, 8))

# # Plot Position
# plt.subplot(3, 1, 1)
# plt.plot(time, trajectory, label="End-effector Position (MPC)")
# plt.axhline(xd[0, 0], color='r', linestyle='--', label="Target Position")
# plt.ylabel("Position")
# plt.legend()

# # Plot Joint Angles
# plt.subplot(3, 1, 2)
# for i in range(joint_angles.shape[1]):
#     plt.plot(time, joint_angles[:, i], label=f"Joint {i + 1} Angle")
# plt.ylabel("Joint Angles [rad]")
# plt.legend()

# # Plot Control Input
# plt.subplot(3, 1, 3)
# plt.step(time[:-1], control_inputs.T, where='post', label="Control Input (Torque)")
# plt.ylabel("Torque")
# plt.xlabel("Time [s]")
# plt.legend()

# plt.tight_layout()
# plt.show()


import numpy as np
import cvxpy as cp

def start_mpc_simulation():
    # Load joint angles from IK
    joint_angles = np.load("arm/joint_angles.npy").flatten()  # shape: (N,)
    n_joints = len(joint_angles)

    T = 0.1   # Sampling time
    N = 10    # MPC prediction horizon

    A = np.array([[0, 1], [0, 0]])  # State matrix
    B = np.array([[0], [1]])        # Input matrix
    A_d = np.eye(2) + T * A
    B_d = T * B
    Q = np.eye(2)
    R = np.eye(1) * 0.1

    smoothed_angles = []

    for i in range(n_joints):
        x0 = np.array([[0], [0]])                  # Initial state: [angle=0, velocity=0]
        xd = np.array([[joint_angles[i]], [0]])    # Desired final angle with 0 velocity

        # Define optimization variables
        x = cp.Variable((2, N + 1))
        u = cp.Variable((1, N))
        cost = 0
        constraints = [x[:, 0] == x0.flatten()]

        for k in range(N):
            cost += cp.quad_form(x[:, k] - xd.flatten(), Q) + cp.quad_form(u[:, k], R)
            constraints += [x[:, k + 1] == A_d @ x[:, k] + B_d @ u[:, k]]
            constraints += [cp.norm(u[:, k], 'inf') <= 10]

        cost += cp.quad_form(x[:, N] - xd.flatten(), Q)

        # Solve the MPC problem
        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve()

        if x.value is not None:
            smoothed_angle = x.value[0, -1]  # Use the final angle
            smoothed_angles.append(smoothed_angle)
        else:
            print(f"⚠️ MPC failed for joint {i}, using original angle.")
            smoothed_angles.append(joint_angles[i])

    smoothed_angles = np.array(smoothed_angles)
    np.save("smoothed_angles.npy", smoothed_angles)
    print("✅ Saved smoothed_angles.npy")

    return smoothed_angles

# Optional run for direct script execution
if __name__ == "__main__":
    start_mpc_simulation()
