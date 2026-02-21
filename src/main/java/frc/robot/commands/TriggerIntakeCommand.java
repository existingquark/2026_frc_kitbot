package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeHopperSubsystem;
import frc.robot.subsystems.IntakeHopperSubsystem.Mode;

/**
 * TriggerIntakeCommand
 *
 * While scheduled:
 * - Requests TRIGGER_INTAKE mode (intake runs, hopper off)
 * - Updates triggerPower from a supplier (usually left trigger axis)
 *
 * This is modular for autonomous because it depends on a supplier, not a controller.
 */
public class TriggerIntakeCommand extends Command {

    // Subsystem that owns the intake/hopper mechanism
    private final IntakeHopperSubsystem intake;

    // Supplies trigger power (0.0 to 1.0)
    private final DoubleSupplier power;

    // Minimum threshold to ignore noise
    private final double deadband;

    /**
     * Constructs a TriggerIntakeCommand.
     *
     * @param intake    intake/hopper subsystem
     * @param power     trigger axis supplier
     * @param deadband  threshold below which power is treated as zero
     */
    public TriggerIntakeCommand(IntakeHopperSubsystem intake, DoubleSupplier power, double deadband) {
        this.intake = intake;
        this.power = power;
        this.deadband = deadband;
        addRequirements(intake);
    }

    /**
     * Called once when scheduled.
     *
     * Selects the correct mode for trigger intake.
     */
    @Override
    public void initialize() {
        intake.requestMode(Mode.TRIGGER_INTAKE);
    }

    /**
     * Called repeatedly while scheduled.
     *
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
     * Called when command ends or is interrupted.
     *
     * Stops motors for safety.
     */
    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    /**
     * Runs until interrupted (typically while held).
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
