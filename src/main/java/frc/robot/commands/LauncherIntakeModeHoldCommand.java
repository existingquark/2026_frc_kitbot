package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * LauncherIntakeModeHoldCommand
 *
 * Purpose:
 * - Runs the shared intake/launcher motor in an intake or outtake direction while scheduled.
 *
 * Key constraint:
 * - This command requires LauncherSubsystem, so it cannot run simultaneously with launcher hold.
 *   That enforces a simple safety interlock automatically.
 */
public class LauncherIntakeModeHoldCommand extends Command {

    private final LauncherSubsystem launcher;
    private final DoubleSupplier percentOutputSupplier;

    /**
     * @param launcher Shared motor subsystem (CAN ID 5)
     * @param percentOutputSupplier Output to apply while active (intake/outtake)
     */
    public LauncherIntakeModeHoldCommand(LauncherSubsystem launcher, DoubleSupplier percentOutputSupplier) {
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
