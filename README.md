# Oscillating Platform 🚁🌊

This ROS 2 package implements a control and predictive estimation system for landing and "surfing" (compensated hovering) operations of a nano-drone on a periodically moving platform (e.g., ships at sea).

The algorithm uses an Auto-Regressive (AR) Model coupled with Recursive Least Squares (RLS), along with a real-time feedforward control action for velocity and position.

## ⚠️ Dependencies and Requirements

* This package was entirely built to operate in conjunction with the **[Crazyswarm2](https://imrclab.github.io/crazyswarm2/)** ecosystem.

* **[TODO]**: Proper operation requires specific topic and *bitmask* configurations in the Crazyswarm2 `crazyflie.yaml` and `teleop.yaml` files (detailed documentation of these parameters will be added in the future).

---

## 📁 Package Structure

```text
oscillating_platform/
├── launch/
│   └── launch.py                    
├── oscillating_platform/            
│   ├── __init__.py                  
│   ├── ar_platform_estimator.py     
│   └── joystick.py                  
├── rviz/
│   └── config.rviz                  
├── package.xml                      
├── setup.py                         
└── README.md

```

### What does each file do?

* **`launch/launch.py`**: Project entry point. It initializes the Crazyswarm2 server, RViz with our custom interface, and both local project nodes simultaneously.

* **`ar_platform_estimator.py`**: The mathematical brain. It fuses TF2 (MoCap) and laser sensor (RangeFinder) data, applies a low-pass filter (EMA), rejects anomalies, and runs the Adaptive Filter (AR + RLS) to predict the future position ($Z$) and velocity ($\dot{Z}$) of the platform.

* **`joystick.py`**: The flight coordinator. A state machine that reads the controller buttons and orchestrates safe transitions. It consumes the estimator's prediction and uses the `/crazyflie/cmd_full_state` topic to inject the Feedforward control into the Crazyflie firmware.

* **`rviz/config.rviz`**: Visual configuration to monitor the current reading (green marker) and the wave prediction horizon (yellow markers with fade-out) in 3D space.

---

## 🚀 How to Build and Run

1. Navigate to the root of your *workspace* (e.g., `ros2_ws`):

```bash
cd ~/ros2_ws

```

2. Build only this package using a symbolic link (so that changes in the Python code do not require recompilation):

```bash
colcon build --symlink-install --packages-select oscillating_platform --cmake-args -DCMAKE_BUILD_TYPE=Release

```

3. Source the terminal environment:

```bash
source install/setup.bash

```

4. Launch the complete system:

```bash
ros2 launch oscillating_platform launch.py

```

### 🎮 Standard Controls (Joystick)

* **Button 0 (A/X)**: Starts Figure-8 flight locked at the current altitude.
* **Button 1 (B/Circle)**: Aborts the current action and returns the drone to the Origin (0.0, 0.0, 0.3m).
* **Button 2 (X/Square)**: Approach routine (Ascends to 1m and moves to the platform's X/Y).
* **Button 3 (Y/Triangle)**: Toggles "Surfing" on/off (Feedforward compensation based on the prediction).
* **Button 12(?/Arrow Down)**: Initiates Dynamic Landing (Time-to-Contact). The drone descends gradually (0.2 m/s) while continuously compensating for the platform's movement, automatically cutting the motors upon expected contact.
