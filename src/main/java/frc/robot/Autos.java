package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.HopperHoldCommand;
import frc.robot.commands.LauncherHoldCommand;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * Autos
 *
 * Purpose:
 * - Central factory for autonomous routines.
 *
 * How it fits in the robot:
 * - RobotContainer selects one of these commands via a SendableChooser.
 * - Robot.autonomousInit() schedules the selected command.
 *
 * Design intent:
 * - Keep autonomous routines as compositions of small commands.
 * - Use time-based steps now; replace with sensor/QuestNav-driven logic later.
 */
public final class Autos {
    private Autos() {}

    /**
     * driveForwardTimed
     *
     * Baseline auto: drive forward for time, then stop.
     * Good first test because it proves autonomous scheduling and drivetrain control work.
     */
    public static Command driveForwardTimed(DriveSubsystem drive) {
        return Commands.sequence(
            // Drive forward for 2 seconds
            Commands.run(() -> drive.arcadeDrive(0.35, 0.0), drive).withTimeout(2.0),

            // Ensure we end stopped
            Commands.runOnce(() -> drive.arcadeDrive(0.0, 0.0), drive)
        );
    }

    /**
     * launcherSpinAndFeedTimed
     *
     * Baseline auto: spin launcher, wait, then feed hopper briefly.
     *
     * Note:
     * - LauncherModeHoldCommand is designed to run forever (mode-hold),
     *   so we MUST apply a timeout when using it in a sequence.
     *
     * Future expansion hooks:
     * - Replace percent output with velocity/RPM control.
     * - Replace WaitCommand timing with "at speed" checks.
     * - Replace fixed setpoints with QuestNav/vision-derived setpoints.
     */
    public static Command launcherSpinAndFeedTimed(
        LauncherSubsystem launcher,
        HopperSubsystem hopper
    ) {
        final double launcherHold = 0.50;
        final double hopperFeed = 0.75;

        return Commands.sequence(
            // Spin launcher for a fixed time (placeholder for "reach speed" logic)
            new LauncherHoldCommand(launcher, () -> launcherHold).withTimeout(1.5),

            // Small gap for stability (optional; can be removed later)
            new WaitCommand(0.2),

            // Feed hopper briefly
            new HopperHoldCommand(hopper, () -> hopperFeed).withTimeout(0.75),

            // Ensure mechanisms end stopped (defensive programming)
            Commands.runOnce(hopper::stop, hopper),
            Commands.runOnce(launcher::stop, launcher)
        );
    }
}
