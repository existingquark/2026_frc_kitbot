package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;

/**
 * HopperHoldCommand
 *
 * Purpose:
 * - Holds the hopper at a requested output while scheduled.
 */
public class HopperHoldCommand extends Command {

    private final HopperSubsystem hopper;
    private final DoubleSupplier percentOutputSupplier;

    public HopperHoldCommand(HopperSubsystem hopper, DoubleSupplier percentOutputSupplier) {
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
