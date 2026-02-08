package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * ArcadeDriveCommand
 *
 * Driver control command for the KITBOT.
 *
 * Control layout (intentional and future-proof):
 * - Left stick Y  : forward / backward
 * - Left stick X  : strafe (RESERVED for future swerve drivetrain)
 * - Right trigger : rotate right (proportional)
 * - Left trigger  : rotate left  (proportional)
 *
 * Mentor notes:
 * - The kitbot cannot strafe, so left stick X is intentionally ignored.
 * - This layout exactly matches our planned competition swerve controls.
 * - Teaching drivers this now avoids retraining later.
 */
public class ArcadeDriveCommand extends Command {

    private final DriveSubsystem drive;
    private final XboxController controller;

    // ============================
    // Tuning constants (demo-safe)
    // ============================
    // Use centralized constant for trigger/stick deadband so tuning is in one place.

    public ArcadeDriveCommand(
        DriveSubsystem drive,
        XboxController controller
    ) {
        this.drive = drive;
        this.controller = controller;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        // ============================
        // Forward / backward
        // ============================
        // Xbox Y axis is inverted: pushing forward returns negative
        double forward = -MathUtil.applyDeadband(
            controller.getLeftY(),
            Constants.TRIGGER_DEADBAND
        );

        // ============================
        // Strafe (reserved for swerve)
        // ============================
        // Kitbot cannot strafe.
        // We read and deadband this axis intentionally to document future use.
       // double strafe = MathUtil.applyDeadband(
       //     controller.getLeftX(),
       //     DEADBAND
        //);
        // NOTE: 'strafe' is not used on the kitbot.

        // ============================
        // Rotation (trigger-based)
        // ============================
        // Right trigger = turn right
        // Left trigger  = turn left
        // Proportional control based on trigger press amount
        double turn = MathUtil.applyDeadband(
            controller.getRightTriggerAxis()
                - controller.getLeftTriggerAxis(),
            Constants.TRIGGER_DEADBAND
        );

        // ============================
        // Drive the robot
        // ============================
        drive.arcadeDrive(forward, turn);
    }

    @Override
    public void end(boolean interrupted) {
        // Always stop the drivetrain when the command ends
        drive.arcadeDrive(0.0, 0.0);
    }
}
