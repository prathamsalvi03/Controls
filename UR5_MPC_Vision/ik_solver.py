import numpy as np
from numpy.linalg import norm, solve
import pinocchio
import os

def solve_ik_ur5(urdf_path, joint_id=6, target_position=np.array([0.01, 0.1, 0.3])):
    root_joint = pinocchio.JointModelFreeFlyer()
    model = pinocchio.buildModelFromUrdf(urdf_path, root_joint)
    data = model.createData()

    oMdes = pinocchio.SE3(np.eye(3), np.array(target_position).reshape(3, 1))

    q = pinocchio.neutral(model)
    eps = 1e-6
    IT_MAX = 1000
    DT = 1e-1
    damp = 1e-12

    for i in range(IT_MAX):
        pinocchio.forwardKinematics(model, data, q)
        iMd = data.oMi[joint_id].actInv(oMdes)
        err = pinocchio.log(iMd).vector

        if norm(err) < eps:
            print("IK convergence achieved.")
            return q

        J = pinocchio.computeJointJacobian(model, data, q, joint_id)
        J = pinocchio.Jlog6(iMd.inverse()) @ J
        v = J.T @ solve(J @ J.T + damp * np.eye(6), err)
        q = pinocchio.integrate(model, q, v * DT)

    print("Warning: IK did not converge.")
    return None

if __name__ == "__main__":
    # Load the target position from vision
    if not os.path.exists("arm/target_position.npy"):
        print("Error: target_position.npy not found.")
        exit(1)

    target_position = np.load("arm/target_position.npy")
    urdf_path = "/home/pratham/Pratham/example-robot-data-master/robots/ur_description/urdf/ur5_robot.urdf"
    joint_id = 6

    joint_angles = solve_ik_ur5(urdf_path, joint_id, target_position)

    if joint_angles is not None:
        np.save("joint_angles.npy", joint_angles)
        print("Joint angles saved to joint_angles.npy")
    else:
        print("Joint angles not saved due to IK failure.")
