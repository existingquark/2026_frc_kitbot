package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * LauncherSubsystem
 *
 * Purpose:
 * - Owns the launcher motor hardware (currently combined with intake/launcher).
 * - Exposes simple control methods used by commands (percent output today).
 *
 * How it fits in the robot:
 * - Created by RobotContainer.
 * - Controlled by commands such as LauncherModeHoldCommand.
 *
 * Key assumptions / constraints:
 * - CAN ID 5 is the "Intake-Launcher" motor (verified in Phoenix Tuner X).
 * - Mechanically coupled systems may exist; software treats this as the launcher motor
 *   for now to keep code modular and easy to iterate.
 *
 * Future expansion hooks:
 * - Replace percent output with velocity control (closed loop).
 * - Accept dynamic setpoints computed from QuestNav localization / vision distance.
 */
public class LauncherSubsystem extends SubsystemBase {

    // ============================
    // Hardware: Launcher Motor (CAN)
    // ============================
    // Use centralized constant for the launcher CAN ID
    private final WPI_VictorSPX launcherMotor = new WPI_VictorSPX(frc.robot.Constants.LAUNCHER_CAN);

    /**
     * Constructor
     *
     * Hardware configuration only:
     * - Neutral mode
     * - Inversion (if needed later)
     *
     * Design note:
     * - Keep logic out of constructors; commands should decide behavior.
     */
    public LauncherSubsystem() {
        // Brake mode holds the mechanism more firmly when output goes to zero.
        launcherMotor.setNeutralMode(NeutralMode.Brake);

        // Inversion is intentionally not set here yet.
        // If direction is wrong, we will set inversion after confirming mechanism direction.
    }

    /**
     * setPercentOutput
     *
     * Runs the launcher motor using open-loop percent output.
     *
     * Inputs:
     * - percent: [-1..1] where positive direction will be defined by mechanism testing.
     *
     * Safety:
     * - Caller is responsible for any interlocks (e.g., not feeding while reversing).
     *
     * Side effects:
     * - Directly commands motor output.
     */
    public void setPercentOutput(double percent) {
        launcherMotor.set(percent);
    }

    /**
     * stop
     *
     * Convenience method to ensure the motor output is zero.
     *
     * Side effects:
     * - Commands motor output to 0.
     */
    public void stop() {
        launcherMotor.set(0.0);
    }

    /**
     * setVelocityRpm (placeholder)
     *
     * Autonomous/QuestNav hook:
     * - Later, this will accept a desired flywheel RPM and run closed-loop velocity control.
     *
     * Note:
     * - Not implemented yet because current hardware is Victor SPX and the mechanism is
     *   in transition (inline Vortex planned). This stub keeps the API design obvious.
     */
    public void setVelocityRpm(double rpm) {
        // TODO: implement once velocity-capable motor controller + sensor strategy is finalized
    }
}
