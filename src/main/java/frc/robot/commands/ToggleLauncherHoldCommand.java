package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeHopperSubsystem;
import frc.robot.subsystems.IntakeHopperSubsystem.Mode;

/**
 * ToggleLauncherHoldCommand
 *
 * Toggles between:
 * - LAUNCHER_HOLD (intake holds at 0.8)
 * - STOP
 *
 * This preserves the original "LauncherOn" behavior but makes it explicit via modes.
 */
public class ToggleLauncherHoldCommand extends Command {

    private final IntakeHopperSubsystem intake;

    public ToggleLauncherHoldCommand(IntakeHopperSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {

        // Toggle behavior: if currently holding, stop; otherwise enable hold mode
        if (intake.getMode() == Mode.LAUNCHER_HOLD) {
            intake.requestMode(Mode.STOP);
        } else {
            intake.requestMode(Mode.LAUNCHER_HOLD);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
