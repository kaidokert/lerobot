#!/usr/bin/env python3
"""
Terminal UI control for SO101 follower arm using blessed.
Works in WSL without X11 or Wayland.
"""

import argparse
import logging
import time
import numpy as np
from pathlib import Path
from blessed import Terminal
from lerobot.robots.so101_follower.so101_follower import SO101FollowerConfig, SO101Follower

# Rerun imports (optional)
try:
    import rerun as rr
    RERUN_AVAILABLE = True
except ImportError:
    RERUN_AVAILABLE = False

# URDF parsing (optional)
try:
    from urdf_parser_py import urdf as urdf_parser
    URDF_AVAILABLE = True
except ImportError:
    URDF_AVAILABLE = False

# Configuration
ROBOT_PORT = "/dev/ttyACM1"
ROBOT_ID = "blue_follower"
STEP_SIZE = 1.0  # How much to move per key press (in degrees)
LARGE_STEP_SIZE = 5.0  # Larger step with Shift

# Joint names for SO101
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

# Setup file logging
LOG_FILE = "blessed_ui.log"


def setup_logging(verbose=False):
    """Setup file logging with optional verbosity."""
    level = logging.DEBUG if verbose else logging.INFO

    # Configure root logger to write to file
    logging.basicConfig(
        level=level,
        format='%(asctime)s [%(levelname)8s] %(name)s:%(lineno)d - %(message)s',
        handlers=[
            logging.FileHandler(LOG_FILE, mode='w'),
        ]
    )

    # Also set lerobot loggers to appropriate level
    logging.getLogger('lerobot').setLevel(level)

    logger = logging.getLogger(__name__)
    logger.info(f"Logging initialized at level {logging.getLevelName(level)}")
    logger.info(f"Log file: {LOG_FILE}")
    return logger


class BlessedKeyboardControl:
    def __init__(self, logger, calibration_dir=None, use_rerun=False, rerun_addr=None, urdf_path=None):
        self.term = Terminal()
        self.logger = logger
        self.use_rerun = use_rerun and RERUN_AVAILABLE
        self.urdf_path = urdf_path
        self.urdf_robot = None

        # Initialize Rerun if requested
        if self.use_rerun:
            if rerun_addr is None:
                rerun_addr = "127.0.0.1:9876"

            # Parse host:port if needed
            if ':' in rerun_addr:
                host, port = rerun_addr.split(':')
            else:
                host = rerun_addr
                port = "9876"

            # Build the proper Rerun connection URL
            rerun_url = f"rerun+http://{host}:{port}/proxy"

            self.logger.info("Initializing Rerun visualization...")
            self.logger.info(f"Connecting to Rerun at: {rerun_url}")

            try:
                rr.init("SO101_Keyboard_Control", spawn=False)
                rr.connect_grpc(rerun_url)
                self.logger.info(f"Rerun connected successfully to {rerun_url}")

                # Load URDF if provided
                if self.urdf_path and URDF_AVAILABLE:
                    self.logger.info(f"Loading URDF from: {self.urdf_path}")
                    try:
                        self.urdf_robot = urdf_parser.URDF.from_xml_file(self.urdf_path)
                        self.logger.info(f"URDF loaded: {self.urdf_robot.name}")
                        self.log_urdf_to_rerun()
                    except Exception as e:
                        self.logger.error(f"Failed to load URDF: {e}", exc_info=True)
                        self.urdf_robot = None
                elif self.urdf_path and not URDF_AVAILABLE:
                    self.logger.warning("URDF path provided but urdf-parser-py not installed. Install with: pip install urdf-parser-py")

            except Exception as e:
                self.logger.error(f"Failed to connect to Rerun: {e}", exc_info=True)
                self.use_rerun = False

        elif use_rerun and not RERUN_AVAILABLE:
            self.logger.warning("Rerun requested but not available. Install with: pip install rerun-sdk")

        # Initialize robot
        print("Connecting to robot...")
        self.logger.info(f"Connecting to robot on {ROBOT_PORT} with id {ROBOT_ID}")

        # Set calibration directory - let the robot use its default path
        # The robot will automatically append /robots/so101_follower/ to the base path
        if calibration_dir is not None:
            calibration_dir = Path(calibration_dir)
            self.logger.info(f"Using custom calibration directory: {calibration_dir}")
        else:
            self.logger.info("Using default calibration directory")

        config = SO101FollowerConfig(
            port=ROBOT_PORT,
            id=ROBOT_ID,
            calibration_dir=calibration_dir  # Will be None if not specified, so robot uses default
        )

        self.robot = SO101Follower(config)

        # Log calibration info BEFORE connecting
        if hasattr(self.robot, 'calibration_fpath'):
            self.logger.info(f"Calibration file path: {self.robot.calibration_fpath}")

        self.robot.connect()
        self.logger.info("Robot connected successfully")

        # Log calibration details AFTER connecting
        if hasattr(self.robot, 'calibration') and self.robot.calibration:
            self.logger.info("Calibration loaded successfully:")
            for motor, calib in self.robot.calibration.items():
                self.logger.info(f"  {motor}: range=[{calib.range_min}, {calib.range_max}], offset={calib.homing_offset}")
        else:
            self.logger.warning("WARNING: No calibration loaded! Robot will use raw values.")

        # Get initial position
        self.logger.info("Reading initial position from robot...")
        obs = self.robot.get_observation()
        self.logger.debug(f"Initial observation: {obs}")
        self.current_positions = {name: obs[f"{name}.pos"] for name in JOINT_NAMES}
        self.logger.info(f"Initial positions: {self.current_positions}")

        # Control state
        self.running = True
        self.selected_joint = 0
        self.last_command = "Ready"
        self.command_history = []
        self.frame_buffer = None  # For reducing flicker

    def draw_bar(self, value, width=20, min_val=-180, max_val=180):
        """Draw a horizontal bar representing a value in degrees."""
        # Normalize value to 0-1 range for display
        normalized = (value - min_val) / (max_val - min_val)
        normalized = np.clip(normalized, 0, 1)
        filled = int(normalized * width)
        bar = "█" * filled + "░" * (width - filled)
        return bar

    def draw_ui(self):
        """Draw the complete UI."""
        t = self.term

        # Build frame in memory first to reduce flicker
        lines = []
        lines.append(t.bold + t.center("SO101 ARM CONTROL") + t.normal)
        lines.append(t.center("=" * 60))
        lines.append("")

        # Robot info
        lines.append(f"  Robot: {t.cyan}{ROBOT_ID}{t.normal} on {t.cyan}{ROBOT_PORT}{t.normal}")
        lines.append("")

        # Joint positions with bars
        lines.append(t.bold + "  JOINT POSITIONS:" + t.normal)
        lines.append("")

        for i, name in enumerate(JOINT_NAMES):
            pos = self.current_positions[name]

            # Highlight selected joint
            if i == self.selected_joint:
                color = t.black_on_green
                marker = " ◄"
            else:
                color = t.normal
                marker = "  "

            # Joint name and number
            lines.append(f"  {color}[{i}] {name:20s}{t.normal}{marker}")

            # Value and bar (assuming degrees from -180 to +180)
            bar = self.draw_bar(pos, width=20, min_val=-180, max_val=180)
            value_color = t.green if -180 <= pos <= 180 else t.red
            lines.append(f"      {value_color}{pos:+7.2f}°{t.normal} │{bar}│")
            lines.append("")

        # Separator
        lines.append("  " + "─" * 60)
        lines.append("")

        # Controls
        lines.append(t.bold + "  CONTROLS:" + t.normal)
        lines.append(f"    {t.yellow}0-5{t.normal}         Select joint")
        lines.append(f"    {t.yellow}↑/W{t.normal}         Increase position (+{STEP_SIZE}°)")
        lines.append(f"    {t.yellow}↓/S{t.normal}         Decrease position (-{STEP_SIZE}°)")
        lines.append(f"    {t.yellow}←/A{t.normal}         Previous joint")
        lines.append(f"    {t.yellow}→/D{t.normal}         Next joint")
        lines.append(f"    {t.yellow}+/-{t.normal}         Large step (±{LARGE_STEP_SIZE}°)")
        lines.append(f"    {t.yellow}R{t.normal}           Read current position from robot")
        lines.append(f"    {t.yellow}H{t.normal}           Home position (all zeros)")
        lines.append(f"    {t.yellow}Q/ESC{t.normal}       Quit")
        lines.append("")

        # Status line
        lines.append("  " + "─" * 60)
        lines.append(f"  Status: {t.cyan}{self.last_command}{t.normal}")

        # Command history (last 3 commands)
        if self.command_history:
            lines.append(f"  History: {t.dim}{' | '.join(self.command_history[-3:])}{t.normal}")

        # Draw everything at once
        with t.hidden_cursor():
            print(t.home + t.clear + '\n'.join(lines), end='', flush=True)

    def add_command(self, cmd):
        """Add a command to history and update status."""
        self.logger.info(f"Command: {cmd}")
        self.last_command = cmd
        self.command_history.append(f"{time.strftime('%H:%M:%S')} {cmd}")

    def send_action(self):
        """Send current positions to robot."""
        action = {f"{name}.pos": self.current_positions[name] for name in JOINT_NAMES}
        self.logger.debug(f"Sending action to robot: {action}")
        try:
            result = self.robot.send_action(action)
            self.logger.debug(f"Action result: {result}")

            # Log to Rerun
            if self.use_rerun:
                self.log_to_rerun()

        except Exception as e:
            self.logger.error(f"Failed to send action: {e}", exc_info=True)
            raise

    def read_position(self):
        """Read current position from robot."""
        self.logger.info("Reading position from robot...")
        try:
            obs = self.robot.get_observation()
            self.logger.debug(f"Observation received: {obs}")
            self.current_positions = {name: obs[f"{name}.pos"] for name in JOINT_NAMES}
            self.logger.info(f"Updated positions: {self.current_positions}")
            self.add_command("Read position from robot")

            # Log to Rerun after reading
            if self.use_rerun:
                self.log_to_rerun()
        except Exception as e:
            self.logger.error(f"Failed to read position: {e}", exc_info=True)
            self.add_command(f"Error reading position: {e}")

    def log_urdf_to_rerun(self):
        """Log URDF static structure to Rerun using Rerun's built-in URDF loader."""
        if not self.use_rerun or not self.urdf_path:
            return

        try:
            self.logger.info("Logging URDF structure to Rerun...")

            # Use Rerun's built-in file loader for URDF
            # This automatically loads the URDF and all associated meshes
            urdf_path_abs = Path(self.urdf_path).resolve()
            self.logger.info(f"Loading URDF file: {urdf_path_abs}")

            # Log the URDF file - Rerun will parse it and load meshes
            rr.log_file_from_path(str(urdf_path_abs), entity_path_prefix="world/robot")
            self.logger.info("URDF logged to Rerun successfully")

        except AttributeError as e:
            self.logger.warning(f"log_file_from_path not available in this Rerun version: {e}")
            self.logger.warning("Try: pip install --upgrade rerun-sdk")
        except Exception as e:
            self.logger.error(f"Failed to log URDF to Rerun: {e}", exc_info=True)

    def update_robot_pose_in_rerun(self):
        """Update robot joint transforms in Rerun based on current joint positions."""
        if not self.use_rerun:
            return

        try:
            import numpy as np

            # Joint angles in radians (convert from degrees)
            joint_angles = {}
            for name in JOINT_NAMES:
                # Convert degrees to radians
                angle_deg = self.current_positions[name]
                angle_rad = np.radians(angle_deg)
                joint_angles[name] = angle_rad

            # When Rerun loads a URDF, it creates the hierarchy as:
            # world/robot/so101_new_calib/base_link/shoulder_pan/shoulder_link/shoulder_lift/...
            # Each joint is a child of its parent link

            robot_prefix = "world/robot/so101_new_calib/base_link"

            # Log transforms for each joint at their correct hierarchical location
            # Joint hierarchy from URDF:
            # base_link -> shoulder_pan -> shoulder_link -> shoulder_lift -> upper_arm_link -> ...

            # NOTE: All joints in the URDF have axis="0 0 1" (Z-axis in their local frame)
            # The actual rotation directions come from the origin transforms in the URDF
            # We need to experiment to match real robot behavior

            # Shoulder pan: URDF says Z-axis, but inverted direction
            rr.log(
                f"{robot_prefix}/shoulder_pan",
                rr.Transform3D(rotation=rr.RotationAxisAngle(axis=[0, 0, -1], angle=joint_angles["shoulder_pan"]))
            )

            # Shoulder lift: Y-axis - this was already correct before!
            rr.log(
                f"{robot_prefix}/shoulder_pan/shoulder_link/shoulder_lift",
                rr.Transform3D(rotation=rr.RotationAxisAngle(axis=[0, 1, 0], angle=joint_angles["shoulder_lift"]))
            )

            # Elbow flex: URDF says Z-axis, needs testing for correct axis
            rr.log(
                f"{robot_prefix}/shoulder_pan/shoulder_link/shoulder_lift/upper_arm_link/elbow_flex",
                rr.Transform3D(rotation=rr.RotationAxisAngle(axis=[0, 0, 1], angle=joint_angles["elbow_flex"]))
            )

            # Wrist flex: URDF says Z-axis, needs testing
            rr.log(
                f"{robot_prefix}/shoulder_pan/shoulder_link/shoulder_lift/upper_arm_link/elbow_flex/lower_arm_link/wrist_flex",
                rr.Transform3D(rotation=rr.RotationAxisAngle(axis=[0, 0, 1], angle=joint_angles["wrist_flex"]))
            )

            # Wrist roll: Y-axis
            rr.log(
                f"{robot_prefix}/shoulder_pan/shoulder_link/shoulder_lift/upper_arm_link/elbow_flex/lower_arm_link/wrist_flex/wrist_link/wrist_roll",
                rr.Transform3D(rotation=rr.RotationAxisAngle(axis=[0, 1, 0], angle=joint_angles["wrist_roll"]))
            )

            # Gripper: Y-axis with inverted sign (axis was correct, just needed sign flip)
            rr.log(
                f"{robot_prefix}/shoulder_pan/shoulder_link/shoulder_lift/upper_arm_link/elbow_flex/lower_arm_link/wrist_flex/wrist_link/wrist_roll/gripper_link/gripper",
                rr.Transform3D(rotation=rr.RotationAxisAngle(axis=[0, -1, 0], angle=joint_angles["gripper"]))
            )

        except Exception as e:
            self.logger.error(f"Failed to update robot pose: {e}", exc_info=True)

    def log_to_rerun(self):
        """Log current state to Rerun."""
        if not self.use_rerun:
            return

        try:
            # Set time for this frame
            current_time = time.time()
            rr.set_time_seconds("timestamp", current_time)

            self.logger.debug(f"Logging to Rerun at time {current_time}")
            self.logger.debug(f"Current positions: {self.current_positions}")

            # Log each joint position as a scalar
            for name in JOINT_NAMES:
                pos = self.current_positions[name]
                self.logger.debug(f"  Logging {name} = {pos}")
                rr.log(f"robot/joints/{name}", rr.Scalars(pos))

            # Log all positions together for easier viewing
            all_positions = [self.current_positions[name] for name in JOINT_NAMES]
            rr.log("robot/all_joints", rr.BarChart(all_positions))

            # Log selected joint indicator
            selected_idx = self.selected_joint
            rr.log("control/selected_joint_index", rr.Scalars(selected_idx))
            rr.log("control/selected_joint_name", rr.TextLog(JOINT_NAMES[self.selected_joint]))

            # Log last command
            if self.last_command:
                rr.log("control/last_command", rr.TextLog(self.last_command))

            # Update robot 3D pose if URDF path is provided
            if self.urdf_path:
                self.update_robot_pose_in_rerun()

            self.logger.debug("Rerun logging completed successfully")

        except Exception as e:
            self.logger.error(f"Failed to log to Rerun: {e}", exc_info=True)

    def home_position(self):
        """Move all joints to zero position."""
        self.logger.info("Moving to home position...")
        for name in JOINT_NAMES:
            self.current_positions[name] = 0.0
        self.send_action()
        self.add_command("Moved to home position (all zeros)")

    def run(self):
        """Main control loop."""
        t = self.term
        self.logger.info("Starting main control loop")

        try:
            with t.fullscreen(), t.cbreak():
                self.logger.debug("Entered fullscreen mode")

                # Log initial state to Rerun
                if self.use_rerun:
                    self.log_to_rerun()

                while self.running:
                    # Draw UI
                    self.draw_ui()

                    # Get key with timeout
                    key = t.inkey(timeout=0.05)  # Reduced timeout for more responsive UI

                    if not key:
                        # Even when no key is pressed, periodically log to Rerun
                        # This helps keep the visualization alive
                        if self.use_rerun:
                            # Only log every ~200ms to avoid spam
                            import random
                            if random.random() < 0.2:  # 20% chance = ~200ms on average
                                self.log_to_rerun()
                        continue

                    self.logger.debug(f"Key pressed: {repr(key)} (name={key.name if hasattr(key, 'name') else None})")
                    joint_name = JOINT_NAMES[self.selected_joint]

                    # Process key
                    if key.name == 'KEY_ESCAPE' or key.lower() == 'q':
                        self.logger.info("Exit key pressed")
                        self.running = False
                        self.add_command("Exiting...")

                    elif key in '012345':
                        self.selected_joint = int(key)
                        self.logger.info(f"Selected joint {self.selected_joint}")
                        self.add_command(f"Selected joint {self.selected_joint}: {JOINT_NAMES[self.selected_joint]}")

                    elif key.name == 'KEY_LEFT' or key.lower() == 'a':
                        self.selected_joint = (self.selected_joint - 1) % len(JOINT_NAMES)
                        self.add_command(f"Selected: {JOINT_NAMES[self.selected_joint]}")

                    elif key.name == 'KEY_RIGHT' or key.lower() == 'd':
                        self.selected_joint = (self.selected_joint + 1) % len(JOINT_NAMES)
                        self.add_command(f"Selected: {JOINT_NAMES[self.selected_joint]}")

                    elif key.name == 'KEY_UP' or key.lower() == 'w':
                        step = LARGE_STEP_SIZE if key.name == 'KEY_SUP' else STEP_SIZE
                        old_pos = self.current_positions[joint_name]
                        self.current_positions[joint_name] += step
                        new_pos = self.current_positions[joint_name]
                        self.send_action()
                        self.add_command(f"{joint_name}: {old_pos:+.3f} → {new_pos:+.3f}")

                    elif key.name == 'KEY_DOWN' or key.lower() == 's':
                        step = LARGE_STEP_SIZE if key.name == 'KEY_SDOWN' else STEP_SIZE
                        old_pos = self.current_positions[joint_name]
                        self.current_positions[joint_name] -= step
                        new_pos = self.current_positions[joint_name]
                        self.send_action()
                        self.add_command(f"{joint_name}: {old_pos:+.3f} → {new_pos:+.3f}")

                    elif key.lower() == 'r':
                        self.read_position()

                    elif key.lower() == 'h':
                        self.home_position()

                    elif key.lower() == '+' or key.lower() == '=':
                        old_pos = self.current_positions[joint_name]
                        self.current_positions[joint_name] += LARGE_STEP_SIZE
                        new_pos = self.current_positions[joint_name]
                        self.send_action()
                        self.add_command(f"{joint_name}: {old_pos:+.3f} → {new_pos:+.3f} (large step)")

                    elif key.lower() == '-' or key.lower() == '_':
                        old_pos = self.current_positions[joint_name]
                        self.current_positions[joint_name] -= LARGE_STEP_SIZE
                        new_pos = self.current_positions[joint_name]
                        self.send_action()
                        self.add_command(f"{joint_name}: {old_pos:+.3f} → {new_pos:+.3f} (large step)")

        except KeyboardInterrupt:
            self.add_command("Interrupted by user")

        finally:
            # Cleanup
            print(t.clear)
            print("Disconnecting robot...")
            self.robot.disconnect()
            print("Done!")


if __name__ == "__main__":
    # Parse arguments
    parser = argparse.ArgumentParser(
        description="SO101 Follower Keyboard Control with Blessed UI",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic usage (no visualization)
  python test_keyboard_control_blessed.py

  # With Rerun visualization (WSL to Windows)
  # First, on Windows run: rerun --connect ws://0.0.0.0:9876
  # Then get WSL host IP: ip route show | grep -i default | awk '{print $3}'
  # Finally on WSL:
  python test_keyboard_control_blessed.py --rerun --rerun-addr <WINDOWS_IP>:9876

  # If WSL2 with mirrored networking (Windows 11 22H2+):
  python test_keyboard_control_blessed.py --rerun --rerun-addr 127.0.0.1:9876
        """
    )
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose debug logging')
    parser.add_argument('--port', default=ROBOT_PORT, help=f'Robot port (default: {ROBOT_PORT})')
    parser.add_argument('--id', default=ROBOT_ID, help=f'Robot ID (default: {ROBOT_ID})')
    parser.add_argument('--calibration-dir', default=None, help='Calibration directory path')
    parser.add_argument('--rerun', action='store_true', help='Enable Rerun visualization')
    parser.add_argument('--rerun-addr', default=None,
                        help='Rerun server address (default: 127.0.0.1:9876). '
                             'For WSL->Windows, use Windows host IP from: ip route show | grep default | awk \'{print $3}\'')
    parser.add_argument('--urdf', default=None,
                        help='Path to URDF file for 3D visualization (e.g., dep/SO-ARM100/Simulation/SO101/so101_new_calib.urdf)')
    args = parser.parse_args()

    # Update configuration from args
    ROBOT_PORT = args.port
    ROBOT_ID = args.id

    # Setup logging
    logger = setup_logging(verbose=args.verbose)
    logger.info(f"Starting SO101 control - Port: {ROBOT_PORT}, ID: {ROBOT_ID}")
    logger.info(f"Verbose mode: {args.verbose}")
    logger.info(f"Rerun visualization: {args.rerun}")
    if args.rerun and args.rerun_addr:
        logger.info(f"Rerun address: {args.rerun_addr}")

    try:
        controller = BlessedKeyboardControl(
            logger,
            calibration_dir=args.calibration_dir,
            use_rerun=args.rerun,
            rerun_addr=args.rerun_addr,
            urdf_path=args.urdf
        )
        controller.run()
    except Exception as e:
        logger.critical(f"Fatal error: {e}", exc_info=True)
        print(f"\nError: {e}")
        print(f"Check {LOG_FILE} for details")
        import traceback
        traceback.print_exc()
