package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeHopperSubsystem;
import frc.robot.subsystems.IntakeHopperSubsystem.Mode;

/**
 * IntakePresetCommand
 *
 * While scheduled:
 * - Requests PRESET_INTAKE mode (intake + hopper together at preset outputs)
 */
public class IntakePresetCommand extends Command {

    private final IntakeHopperSubsystem intake;

    public IntakePresetCommand(IntakeHopperSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.requestMode(Mode.PRESET_INTAKE);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
