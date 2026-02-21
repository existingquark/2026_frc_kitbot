package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * LauncherHoldCommand
 *
 * Purpose:
 * - Holds the launcher in an "active mode" while scheduled.
 * - This is a state-holding command (continuous enforcement), not a one-shot action.
 */
public class LauncherHoldCommand extends Command {

    // Subsystem this command controls
    private final LauncherSubsystem launcher;

    // Setpoint source for launcher demand (percent output for now)
    private final DoubleSupplier percentOutputSupplier;

    public LauncherHoldCommand(LauncherSubsystem launcher, DoubleSupplier percentOutputSupplier) {
        this.launcher = launcher;
        this.percentOutputSupplier = percentOutputSupplier;
        addRequirements(launcher);
    }

    @Override
    public void execute() {
        launcher.setPercentOutput(percentOutputSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        launcher.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
