package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * LauncherModeHoldCommand
 *
 * Purpose:
 * - Holds the launcher in an "active mode" while scheduled.
 * - This is a state-holding command (continuous enforcement), not a one-shot action.
 *
 * How it fits in the robot:
 * - Scheduled by RobotContainer based on a button (toggle or hold).
 * - Requires LauncherSubsystem so no other command can control the launcher simultaneously.
 *
 * Key assumptions / constraints:
 * - Uses percent output today.
 * - The setpoint is provided via a supplier so we can swap sources later:
 *   - constant value for teleop testing
 *   - dynamic value derived from QuestNav / vision distance in the future
 *
 * Future expansion hooks:
 * - Replace percent output with velocity hold (RPM).
 * - Use QuestNav distance-to-goal mapping to compute the setpoint.
 */
public class LauncherModeHoldCommand extends Command {

    // Subsystem this command controls
    private final LauncherSubsystem launcher;

    // Setpoint source for launcher demand (percent output for now)
    private final DoubleSupplier percentOutputSupplier;

    /**
     * Constructor
     *
     * @param launcher Subsystem required by this command (prevents conflicts)
     * @param percentOutputSupplier Provides the desired percent output while running
     */
    public LauncherModeHoldCommand(LauncherSubsystem launcher, DoubleSupplier percentOutputSupplier) {
        this.launcher = launcher;
        this.percentOutputSupplier = percentOutputSupplier;

        // Claims exclusive control of the launcher while scheduled
        addRequirements(launcher);
    }

    @Override
    public void initialize() {
        // Initialization is intentionally light; the command "holds" via execute().
    }

    @Override
    public void execute() {
        // Continuously enforce the launcher output (required for MotorSafety-style patterns).
        launcher.setPercentOutput(percentOutputSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        // Always stop the launcher when command ends to avoid unintended motion.
        launcher.stop();
    }

    @Override
    public boolean isFinished() {
        // This is a mode-hold command; it runs until canceled (toggle off or replaced).
        return false;
    }
}
