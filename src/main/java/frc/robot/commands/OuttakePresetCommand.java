package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeHopperSubsystem;
import frc.robot.subsystems.IntakeHopperSubsystem.Mode;

/**
 * OuttakePresetCommand
 *
 * While scheduled:
 * - Requests PRESET_OUTTAKE mode (reverse intake + hopper at preset outputs)
 */
public class OuttakePresetCommand extends Command {

    private final IntakeHopperSubsystem intake;

    public OuttakePresetCommand(IntakeHopperSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.requestMode(Mode.PRESET_OUTTAKE);
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
