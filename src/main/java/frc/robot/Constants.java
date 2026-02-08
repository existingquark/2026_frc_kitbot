package frc.robot;

//import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

/**
 * Robot-wide numerical or boolean constants.
 *
 * Purpose:
 * - Centralize hardware CAN IDs and tuning constants so they are easy to find and change.
 * - Prevent magic numbers scattered through the codebase.
 *
 * Usage:
 * - Refer to these values from `RobotContainer` and subsystems (e.g. `Constants.INTAKE_LAUNCHER_CAN`).
 */
public final class Constants {

	// CAN IDs
	// ID 5: currently the intake/launcher motor (Phoenix tuner verified)
	// ID 1: hopper motor

	// Drive CAN IDs (keep these centralized so wiring changes are easy)
	public static final int LEFT_FRONT_CAN = 4;
	public static final int LEFT_BACK_CAN = 6;
	public static final int RIGHT_FRONT_CAN = 3;
	public static final int RIGHT_BACK_CAN = 2;

	// NOTE: The robot hardware currently uses a combined intake/launcher motor
	// on CAN ID 5. When a dedicated launcher motor is installed, update
	// `LAUNCHER_CAN` to that device's ID and `INTAKE_CAN` to the intake's ID.
	// For now both are set to the existing ID to preserve behavior.
	public static final int LAUNCHER_CAN = 5;
	public static final int INTAKE_CAN = 5; // change when separate intake motor is wired

	// CAN ID for the hopper motor controller
	public static final int HOPPER_CAN = 1;

	// Operator tuning
	// Deadband applied to trigger axes to avoid small noise-driven outputs
	// (trigger values with absolute value below this are treated as zero)
	public static final double TRIGGER_DEADBAND = 0.05;

	// Controller USB port mapping (WPILib uses the index from Driver Station)
	// Change these if the physical controllers are plugged into different USB ports.
	public static final int DRIVER_CONTROLLER_PORT = 0;
	public static final int OPERATOR_CONTROLLER_PORT = 1;

	// Teleop tunables (move here so students can change without editing RobotContainer)
	// Fractional motor outputs in range [-1.0, 1.0]. Signs may need flipping
	// depending on wiring/mechanism direction.

	// Launcher hold power (used by LauncherModeHoldCommand)
	public static final double LAUNCHER_HOLD_POWER = 0.50;
	// Hopper manual feed forward power
	public static final double HOPPER_FORWARD_POWER = 0.75;
	// Hopper manual feed reverse power (negative to reverse direction)
	public static final double HOPPER_REVERSE_POWER = -0.75;
	// Intake/outtake motor powers for the shared launcher-intake motor
	public static final double INTAKE_POWER = 0.75;
	public static final double OUTTAKE_POWER = -0.50;

	// Intake / Hopper preset powers (moved from code into constants for tuning)
	public static final double PRESET_INTAKE_INTAKE_POWER = 0.75;
	public static final double PRESET_INTAKE_HOPPER_POWER = 0.90;
	public static final double PRESET_OUTTAKE_INTAKE_POWER = -0.50;
	public static final double PRESET_OUTTAKE_HOPPER_POWER = -0.70;
	//public static final double LAUNCHER_HOLD_INTAKE_POWER = 0.80; do i need this still???

	// Drive configuration
	// Invert the right side group so positive forward values drive forwards
	public static final boolean RIGHT_SIDE_INVERTED = true;

	// Fixed rotation power used when bumpers are held (range 0..1).
	// Use bumpers for coarse rotation; left bumper rotates left, right bumper rotates right.
	public static final double TURN_POWER = 0.50;

	private Constants() { /* prevent instantiation */ }
}
