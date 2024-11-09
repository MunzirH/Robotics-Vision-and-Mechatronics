# 🤖 Robotics, Vision, and Mechatronics for Manufacturing

Welcome to the **Robotics, Vision, and Mechatronics for Manufacturing** repository! This collection of MATLAB scripts dives into robotics fundamentals and advanced techniques, covering everything from kinematics and vision-based trajectory planning to visual servo control and error analysis. Here, you’ll find code for the Puma 560 and UR5e robotic arms, exploring how they interact with the world around them. 🌐

## 📂 Contents

### 🔬 Labs
1. **Lab 1**: **Trajectory Generation and Grasping with Puma 560**
   - 🚀 Implements trajectory planning for the Puma 560 robot, moving between different configurations to grasp objects.
   - 🛠️ Covers both joint-space and Cartesian trajectories with visualizations of robot motion.
   - 📏 Analyzes the impact of manufacturing errors on the end-effector's accuracy.

2. **Lab 2**: **Camera Calibration and Motion Control**
   - 🎥 Calibrates a camera mounted on a UR5e robot to enable accurate pose estimation and trajectory tracking.
   - ⚙️ Explores open-loop and closed-loop motion control in MATLAB Simulink, analyzing stability under various conditions.

3. **Lab 3**: **Vision-Based Trajectory Planning**
   - ✍️ Calculates a vision-based trajectory for the Puma 560 robot to draw a rectangle on a whiteboard using a marker.
   - 🖼️ Uses camera calibration and transformations to generate precise motion based on visual input.

### 🎓 Final Project
- **Visual Servo Control with PBVS and IBVS**
   - 🎯 Develops Position-Based Visual Servo (PBVS) and Image-Based Visual Servo (IBVS) control algorithms to guide a camera to a target pose using visual feedback.
   - 🔍 Compares the robustness of PBVS and IBVS under noisy conditions, analyzing sensitivity to depth variations.
   - 📈 Includes visualization of feature trajectory, camera velocity, and error convergence in 3D space.

## 🛠️ Requirements
- **MATLAB**: This repository is built for MATLAB and requires the following toolboxes:
  - [Robotics Toolbox by Peter Corke](https://petercorke.com/toolboxes/robotics-toolbox/) for robotic manipulations and transformations.
  - **Computer Vision Toolbox** for camera calibration and pose estimation.
  - **Statistics and Machine Learning Toolbox** for adding Gaussian noise in vision-based simulations.

## 🚀 Usage
Each lab and project has its own script. Load the required toolboxes and follow the instructions within each script to reproduce the experiments and visualizations. This code is structured to explore different robotics and vision scenarios, making it a fun and educational resource for learning real-world applications of robotics. 🤓
