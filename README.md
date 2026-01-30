Markdown
# ROS2 Self-Balancing Robot Simulation

A PID-controlled self-balancing robot simulated in CoppeliaSim, communicating with a ROS2 Python node.

## 🎥 Demo
[Insert a GIF or Screenshot of your robot balancing here]

## 🛠️ Requirements
* Ubuntu 20.04 / 22.04
* ROS2 (Humble/Foxy)
* CoppeliaSim Edu

## 🚀 How to Run

1. **Clone the Repo:**
   ```bash
   cd ~/ros2_ws/src
   git clone [https://github.com/YOUR_USERNAME/self_balancing_robot.git](https://github.com/YOUR_USERNAME/self_balancing_robot.git)
   cd ~/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
Launch CoppeliaSim:

Open CoppeliaSim.

Load the scene: File > Open Scene > scenes/balancing_robot.ttt

Press Play.

Start the Brain:

Bash
ros2 run self_balancing_brain pid_brain
🧠 Control Logic
Lua Script: Handles sensor reading (IMU) and motor velocity control.

Python Node: Runs a PID loop to calculate torque based on tilt angle.


### 5. Final Checklist before Pushing
1.  [ ] Did you copy the **text version** of the Lua script?
2.  [ ] Did you enable `target_angle` trim in your Python code?
3.  [ ] Did you include the `.gitignore` so you don't upload the `build` folder?