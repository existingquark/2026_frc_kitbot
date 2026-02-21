package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * LauncherIntakeHoldCommand
 *
 * Purpose:
 * - Runs the shared intake/launcher motor in an intake or outtake direction while scheduled.
 */
public class LauncherIntakeHoldCommand extends Command {

    private final LauncherSubsystem launcher;
    private final DoubleSupplier percentOutputSupplier;

    public LauncherIntakeHoldCommand(LauncherSubsystem launcher, DoubleSupplier percentOutputSupplier) {
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
