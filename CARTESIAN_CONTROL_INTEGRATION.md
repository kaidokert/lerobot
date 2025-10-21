# Cartesian Control Integration

## Overview

The keyboard control script (`test_keyboard_control_blessed.py`) has been enhanced with forward and inverse kinematics support, enabling Cartesian coordinate control in addition to joint angle control.

## Features Added

### 1. Dual Control Modes
- **Joint Mode** (default): Direct control of individual joint angles
- **Cartesian Mode**: Control end-effector position in X/Y/Z coordinates using inverse kinematics

### 2. Mode Switching
- Press **M** key to toggle between Joint and Cartesian modes
- Mode is displayed in the UI with color coding:
  - Joint mode: Cyan
  - Cartesian mode: Green

### 3. Cartesian Controls
When in Cartesian mode:
- **W/S**: Move along X-axis (±10mm steps)
- **A/D**: Move along Y-axis (±10mm steps)
- **↑/↓**: Move along Z-axis (±10mm steps)
- **+/-**: Larger steps (±50mm)
- **6**: Toggle gripper (open/close)

### 4. End-Effector Position Display
When in Cartesian mode with IK enabled, the UI shows:
- Current end-effector X, Y, Z position in meters
- Live updates as the robot moves

### 5. Rerun Visualization Enhancement
- End-effector pose is logged to Rerun as a 3D point and transform
- Visible in the Rerun viewer at `robot/ee_position` and `robot/ee_pose`
- Control mode is logged to `control/mode`

## Installation Requirements

### Basic Usage (Joint Mode Only)
No additional dependencies needed beyond what was already installed.

### Cartesian Control (IK Mode)
Requires the `placo` library:

```bash
cd /home/kaido/code/lerobot
pip install -e ".[kinematics]"
```

## Usage

### Enable IK Support
```bash
# With IK enabled (requires placo installation)
python test_keyboard_control_blessed.py \
  --rerun \
  --rerun-addr 192.168.5.222:9876 \
  --urdf dep/SO-ARM100/Simulation/SO101/so101_new_calib.urdf \
  --enable-ik

# Save settings including IK
python test_keyboard_control_blessed.py \
  --enable-ik \
  --save-config
```

### Without IK (Joint Mode Only)
```bash
# Works without placo, just no Cartesian mode
python test_keyboard_control_blessed.py \
  --rerun \
  --rerun-addr 192.168.5.222:9876 \
  --urdf dep/SO-ARM100/Simulation/SO101/so101_new_calib.urdf
```

## Configuration

The `--enable-ik` flag is now saved in the config file (`~/.config/lerobot/keyboard_control.json`):

```json
{
  "port": "/dev/ttyACM1",
  "id": "blue_follower",
  "rerun": true,
  "rerun_addr": "192.168.5.222:9876",
  "urdf": "dep/SO-ARM100/Simulation/SO101/so101_new_calib.urdf",
  "enable_ik": true
}
```

## Implementation Details

### Methods Added

1. **`get_current_ee_pose()`**
   - Uses forward kinematics to compute current end-effector pose
   - Returns 4x4 transformation matrix
   - Uses first 5 joints (excludes gripper)

2. **`move_cartesian(delta_x, delta_y, delta_z)`**
   - Applies Cartesian deltas to current end-effector position
   - Solves inverse kinematics to find required joint angles
   - Updates joints and sends commands to robot
   - Returns True on success, False on IK failure

3. **UI Updates**
   - Enhanced `draw_ui()` to show mode and end-effector position
   - Different control instructions based on current mode
   - Mode indicator with color coding

4. **Key Handling**
   - Mode-specific key handlers in main control loop
   - Graceful fallback if IK not available
   - Clear error messages when placo is missing

### Kinematics Configuration
- **URDF**: `dep/SO-ARM100/Simulation/SO101/so101_new_calib.urdf`
- **Target Frame**: `gripper_frame_link` (end-effector)
- **Active Joints**: First 5 joints (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll)
- **Gripper**: Excluded from IK, controlled independently
- **IK Weights**: position_weight=1.0, orientation_weight=0.01

### Cartesian Step Sizes
- **Normal step**: 10mm (0.01m)
- **Large step**: 50mm (0.05m) - activated with +/- keys

## Error Handling

- If placo is not installed and user tries to enable IK:
  - Warning message displayed
  - Mode switching disabled with helpful error message

- If IK solving fails (unreachable position):
  - Error logged
  - "Cartesian move failed (IK error)" message shown
  - Robot position unchanged

## Testing

### Without placo (Joint Mode Only)
1. Start script without `--enable-ik`
2. Verify joint mode controls work
3. Press 'M' - should show message about needing placo

### With placo (Full Cartesian Support)
1. Install placo: `pip install -e ".[kinematics]"`
2. Start script with `--enable-ik --urdf <path>`
3. Press 'M' to switch to Cartesian mode
4. Verify end-effector position display appears
5. Test W/A/S/D and arrow keys for Cartesian movement
6. Check Rerun viewer for end-effector visualization

## Files Modified

- **`test_keyboard_control_blessed.py`**: Main implementation
  - Added kinematics methods (lines 432-481)
  - Enhanced UI with mode display (lines 234-326)
  - Added mode switching and Cartesian key handling (lines 609-727)
  - Added `--enable-ik` argument (line 780-782)

## Next Steps (Optional Enhancements)

1. **Orientation Control**: Add rotation control (roll/pitch/yaw)
2. **Adjustable Step Size**: Runtime adjustment of Cartesian step sizes
3. **Position Presets**: Save/recall favorite positions
4. **Path Recording**: Record and replay Cartesian trajectories
5. **Collision Avoidance**: Integrate workspace limits and safety checks
