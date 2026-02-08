package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeHopperSubsystem;
import frc.robot.subsystems.IntakeHopperSubsystem.Mode;

/**
 * TriggerOuttakeCommand
 *
 * While scheduled:
 * - Requests TRIGGER_OUTTAKE mode (hopper runs, intake off)
 * - Updates triggerPower from a supplier (usually right trigger axis)
 */
public class TriggerOuttakeCommand extends Command {

    private final IntakeHopperSubsystem intake;
    private final DoubleSupplier power;
    private final double deadband;

    /**
     * Constructs a TriggerOuttakeCommand.
     *
     * @param intake    intake/hopper subsystem
     * @param power     trigger axis supplier
     * @param deadband  threshold below which power is treated as zero
     */
    public TriggerOuttakeCommand(IntakeHopperSubsystem intake, DoubleSupplier power, double deadband) {
        this.intake = intake;
        this.power = power;
        this.deadband = deadband;
        addRequirements(intake);
    }

    /**
     * Selects the correct mode for trigger outtake.
     */
    @Override
    public void initialize() {
        intake.requestMode(Mode.TRIGGER_OUTTAKE);
    }

    /**
     * Updates the trigger power parameter.
     */
    @Override
    public void execute() {
        double p = power.getAsDouble();

        // Deadband to prevent accidental creeping
        if (p < deadband) p = 0.0;

        intake.setTriggerPower(p);
    }

    /**
     * Stops motors when command ends.
     */
    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
