Reference Paper : Model Predictive Control Design of a 3-DOF Robot Arm Based on Recognition of Spatial Coordinates


# UR5_MPC_Vision 🚀

This repository demonstrates a complete pipeline for **real-time robot control** of a UR5 arm in simulation. It integrates **computer vision**, **inverse kinematics (IK)**, **Model Predictive Control (MPC)**, and **PyBullet simulation** to detect an object, compute its 3D position, generate smooth joint motions, and visualize execution.

---

## 🧩 Pipeline Overview

### 🔍 Vision Module (`Vision_code.py` / `.ipynb`)
- Uses **OpenCV** and **HSV color filtering** to detect a target object.
- Performs **camera calibration** using a known-width object at a known distance.
- Computes the object's **3D position in space** and saves it as `target_position.npy`.

### 🤖 Inverse Kinematics (`ik_solver.py`)
- Loads `target_position.npy` and solves for **UR5 joint angles** using **Pinocchio** IK.
- Saves the resulting angles to `joint_angles.npy`.

### 🧠 Model Predictive Control (`MPC.py`)
- Loads `joint_angles.npy` and applies **MPC** to smooth each joint's trajectory.
- Outputs `smoothed_angles.npy` containing time-optimized joint angles.

### 🕹️ Simulation Module (`pybullet_visualizer.py`)
- Loads `smoothed_angles.npy` and animates the **UR5 arm** in a **PyBullet** environment.
- Visualizes smooth, targeted motion of the robot arm.

---

## 📁 Repository Structure

UR5_MPC_Vision/
│
├─ Vision_code.py(.ipynb)      # Target detection, depth, & 3D position → target_position.npy
├─ ik_solver.py                # IK solver → joint_angles.npy
├─ MPC.py                      # MPC smoother → smoothed_angles.npy
├─ pybullet_visualizer.py      # PyBullet UR5 simulation using smoothed_angles.npy
│
└─ README.md                   # 📝 You are here


---

## 🛠️ Setup Instructions

1. **Clone this repo**  

   git clone https://github.com/prathamsalvi03/Controls.git
   cd Controls/UR5_MPC_Vision
  

2. **Install dependencies**  

   pip install numpy opencv-python pybullet pinocchio cvxpy matplotlib
 

3. **Download UR5 URDF**  
   Ensure the UR5 robot description is available at the correct path (update `urdf_path` variables as needed).

---

## 🏃‍♂️ How to Run

Execute the modules in sequence:

1. **Detect & save 3D position**  
   
   python3 Vision_code.py
 

2. **Solve IK and save joint configuration**  
   
   python3 ik_solver.py
  

3. **Smooth joint trajectories with MPC**  
  
   python3 MPC.py
   ```

4. **Visualize UR5 motion in PyBullet**  
   
   python3 pybullet_visualizer.py
   ```

---

## 📌 Important Notes

- The vision module expects a colored object (default hue range set for red).
- Units: depth is estimated in **meters**; IK and PyBullet expect meters.
- MPC setup: treats each joint independently using a simple discrete-time model.
- Adjust module parameters (HSV thresholds, focal length, horizon length, weights) based on your use case.

---

## 🔧 Customization Tips

- **Change color target**: Modify `lower_hsv` / `upper_hsv` in `Vision_code.py`.
- **URDF path**: Ensure all scripts reference the correct URDF location.
- **Tweak MPC behavior**: Change horizon `N`, sampling `T`, or cost matrices `Q`, `R` in `MPC.py`.

---

## 🎯 Expected Output

- A series of `.npy` files tracking the state:  
  `target_position.npy` → `joint_angles.npy` → `smoothed_angles.npy`
- PyBullet window visualizing smooth UR5 arm movement toward the detected object.

---

## 🫂 Want to Contribute?

- ✅ **Issue tracking**: For bug reporting and feature requests.
- ✅ **Pull requests**: Open welcome—please follow standard Git commit message format.
- ✅ **Feature Suggestions**: Ideas like ROS integration, real sensor input, or real robot control.

---

## 📄 License

MIT License — feel free to use and modify!

---

## 🙋‍♂️ Contact

Developed by **Pratham Salvi** — [GitHub profile](https://github.com/prathamsalvi03).  
Reach out for collaboration or support.
