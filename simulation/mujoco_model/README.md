# MuJoCo Arm Model

This directory contains the MuJoCo MJCF model for the 2-DOF robotic arm.

## Files

- `arm.xml` - Main MuJoCo model definition
- `visualize_model.py` - Interactive visualization script

## Model Specifications

### Physical Structure
- **Base**: Fixed platform (5cm x 5cm x 2cm box)
- **Link 1**: Shoulder to elbow (10cm capsule)
- **Link 2**: Elbow to end-effector (8cm capsule)
- **End Effector**: Spherical tip with IMU sensor site

### Joints
- **Shoulder**: Revolute joint, Z-axis rotation, range [-90°, +90°]
- **Elbow**: Revolute joint, Y-axis rotation, range [-90°, +90°]

### Actuators
- Position servos modeling SG90 servo behavior (kp=2.0)

### Sensors
- Joint position sensors for both joints
- IMU orientation sensor (quaternion) at end-effector

## Testing

The model includes basic XML validation tests that run without MuJoCo:

```bash
cd ../
pytest tests/test_mujoco_model.py -v
```

### MuJoCo Installation Note

**Python 3.14 Compatibility Issue**: The current MuJoCo Python package does not have pre-built wheels for Python 3.14. To run the full test suite and visualization:

1. **Option 1**: Use Python 3.11 or 3.12 (recommended)
2. **Option 2**: Build MuJoCo from source (requires setting MUJOCO_PATH environment variable)
3. **Option 3**: Wait for official Python 3.14 wheels

For now, basic XML validation tests will run and pass. MuJoCo-dependent tests are automatically skipped.

## Visualization

Once MuJoCo is installed, run:

```bash
python visualize_model.py
```

This launches an interactive 3D viewer showing the arm model. Press ESC to close.
