"""Visualize the arm model interactively."""

import mujoco
import mujoco.viewer
import numpy as np

# Load model
model = mujoco.MjModel.from_xml_path('arm.xml')
data = mujoco.MjData(model)

# Set initial joint angles
data.qpos[0] = 0.5   # Shoulder: 30 degrees
data.qpos[1] = -0.3  # Elbow: -17 degrees

# Launch viewer
print("Launching MuJoCo viewer...")
print("Press ESC to close")
mujoco.viewer.launch(model, data)
