import pybullet as p
import pybullet_data
import numpy as np
import time
from numpy.linalg import norm, solve
import pinocchio

class RobotIKVisualizer:
    def __init__(self, urdf_path):
        # PyBullet setup
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)  # Disable real-time simulation for better control
        
        # Load ground plane
        self.plane_id = p.loadURDF("plane.urdf")
        
        # Load robot in Pinocchio
        self.model = pinocchio.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        
        # Load robot in PyBullet
        start_pos = [0, 0, 0]
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.robot_id = p.loadURDF(urdf_path,
                                 basePosition=start_pos,
                                 baseOrientation=start_orientation,
                                 useFixedBase=True)
        
        # Set up camera
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=0,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0.5]
        )
        
        # Get number of joints
        self.num_joints = p.getNumJoints(self.robot_id)
        print(f"Number of joints: {self.num_joints}")
        
        # Print joint information
        for i in range(self.num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            print(f"Joint {i}: {joint_info[1].decode('utf-8')}")
        
        # IK parameters
        self.JOINT_ID = self.num_joints - 1  # End effector joint (typically last joint)
        self.eps = 1e-4    # Error threshold
        self.IT_MAX = 1000  # Maximum iterations
        self.DT = 1e-1     # Time step
        self.damp = 1e-12  # Damping factor

    def solve_ik(self, target_position):
        """Solve IK for given target position."""
        # Create target transform
        oMdes = pinocchio.SE3(np.eye(3), np.array(target_position))
        
        # Start from neutral configuration
        q = pinocchio.neutral(self.model)
        
        # Iterative IK solving
        for i in range(self.IT_MAX):
            pinocchio.forwardKinematics(self.model, self.data, q)
            iMd = self.data.oMi[self.JOINT_ID].actInv(oMdes)
            
            err = pinocchio.log(iMd).vector
            if norm(err) < self.eps:
                print("IK converged!")
                return q
                
            J = pinocchio.computeJointJacobian(self.model, self.data, q, self.JOINT_ID)
            J = pinocchio.Jlog6(iMd.inverse()) @ J
            v = J.T.dot(solve(J.dot(J.T) + self.damp * np.eye(6), err))
            q = pinocchio.integrate(self.model, q, v * self.DT)
            
            if i % 100 == 0:
                print(f"Iteration {i}, error: {norm(err)}")
        
        print("Warning: IK did not converge")
        return q

    def visualize_solution(self, joint_angles):
        """Visualize the solution in PyBullet."""
        for i in range(len(joint_angles)):
            p.setJointMotorControl2(
                self.robot_id,
                i,
                p.POSITION_CONTROL,
                targetPosition=joint_angles[i],
                force=100
            )
        p.stepSimulation()

    def run_interactive(self):
        """Run interactive loop for coordinate input."""
        while True:
            try:
                # Get target position from user
                print("\nEnter target position (x y z) or 'q' to quit:")
                user_input = input()
                
                if user_input.lower() == 'q':
                    break
                    
                # Parse coordinates
                x, y, z = map(float, user_input.split())
                target_position = [x, y, z]
                
                # Solve IK
                solution = self.solve_ik(target_position)
                
                # Visualize solution
                self.visualize_solution(solution)
                
            except ValueError:
                print("Invalid input. Please enter three numbers separated by spaces.")
            except Exception as e:
                print(f"Error: {e}")

    def cleanup(self):
        """Cleanup PyBullet connection."""
        p.disconnect()

if __name__ == "__main__":
    # Replace this with your URDF path
    urdf_path = "/home/pratham/Pratham/example-robot-data-master/robots/ur_description/urdf/ur5_robot.urdf"
    visualizer = RobotIKVisualizer(urdf_path)
    try:
        visualizer.run_interactive()
    finally:
        visualizer.cleanup()