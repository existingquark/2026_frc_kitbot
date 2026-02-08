package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;

/**
 * HopperModeHoldCommand
 *
 * Purpose:
 * - Holds the hopper at a requested output while scheduled.
 * - Used for simple feed/reverse modes (teleop and autonomous).
 *
 * Design note:
 * - Supplier allows easy swap from constant values to sensor/auto logic later.
 */
public class HopperModeHoldCommand extends Command {

    private final HopperSubsystem hopper;
    private final DoubleSupplier percentOutputSupplier;

    /**
     * @param hopper Hopper subsystem this command controls
     * @param percentOutputSupplier Desired output while the command is active
     */
    public HopperModeHoldCommand(HopperSubsystem hopper, DoubleSupplier percentOutputSupplier) {
        this.hopper = hopper;
        this.percentOutputSupplier = percentOutputSupplier;
        addRequirements(hopper);
    }

    @Override
    public void execute() {
        hopper.setPercentOutput(percentOutputSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        hopper.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
