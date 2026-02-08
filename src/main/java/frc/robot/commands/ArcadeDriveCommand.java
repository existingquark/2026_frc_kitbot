package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCommand extends Command {

    private final DriveSubsystem drive;
    private final CommandXboxController controller;

    private final SlewRateLimiter fwdLimiter =
        new SlewRateLimiter(Constants.DRIVE_FWD_SLEW_RATE);

    private final SlewRateLimiter turnLimiter =
        new SlewRateLimiter(Constants.DRIVE_TURN_SLEW_RATE);

    public ArcadeDriveCommand(DriveSubsystem drive, CommandXboxController controller) {
        this.drive = drive;
        this.controller = controller;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        // Forward/back (stick Y inverted)
        double forward = -MathUtil.applyDeadband(
            controller.getLeftY(),
            Constants.DRIVE_STICK_DEADBAND
        );

        // Turn (trigger difference)
        double turn = MathUtil.applyDeadband(
            controller.getRightTriggerAxis() - controller.getLeftTriggerAxis(),
            Constants.DRIVE_TRIGGER_DEADBAND
        );

        if (Constants.DRIVE_SQUARE_INPUTS) {
            forward = Math.copySign(forward * forward, forward);
            turn = Math.copySign(turn * turn, turn);
        }

        // Precision mode while holding RB
        if (controller.rightBumper().getAsBoolean()) {
            forward *= Constants.DRIVE_PRECISION_SCALE;
            turn *= Constants.DRIVE_PRECISION_SCALE;
        }

        // Smooth the outputs
        forward = fwdLimiter.calculate(forward);
        turn = turnLimiter.calculate(turn);

        drive.arcadeDrive(forward, turn);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
