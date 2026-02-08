package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.HopperHoldCommand;
import frc.robot.commands.LauncherIntakeHoldCommand;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * RobotContainer
 *
 * Design goals:
 * - DRIVER only drives
 * - OPERATOR controls manipulators
 * - Operator mapping matches the agreed control spec (toggles + manual triggers)
 * - Keep behavior stable so Spark Flex changes do NOT alter button mapping
 */
public class RobotContainer {

    // ============================
    // Controllers
    // ============================
    private final CommandXboxController driverController =
        new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);

    private final CommandXboxController operatorController =
        new CommandXboxController(Constants.OPERATOR_CONTROLLER_PORT);

    // ============================
    // Subsystems
    // ============================
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final HopperSubsystem hopperSubsystem = new HopperSubsystem();
    private final LauncherSubsystem launcherSubsystem = new LauncherSubsystem();

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
    }

    // ============================
    // Default Commands
    // ============================
    private void configureDefaultCommands() {
        // ✅ Driver drives (NOT operator)
        driveSubsystem.setDefaultCommand(
            new ArcadeDriveCommand(driveSubsystem, driverController)
        );
    }

    // ============================
    // Button Bindings
    // ============================
    private void configureButtonBindings() {

        // ====================================================
        // OPERATOR CONTROLS (Controller 1)
        // ====================================================
        // Spec:
        // - RB toggle: automatic intake + shoot from ground
        // - RT hold:   manual proportional intake + shoot from ground
        // - LB toggle: automatic outtake + shoot from holding
        // - LT hold:   manual proportional outtake + shoot from holding
        // - X: transfer to holding
        // - A: spins holding in (retain)
        // - Y: spit out (eject)
        // - B: intentionally left open for now

        // RB (toggle): Auto ground intake + shoot (launcher + hopper forward)
        operatorController.rightBumper()
            .toggleOnTrue(
                new ParallelCommandGroup(
                    new LauncherIntakeHoldCommand(
                        launcherSubsystem,
                        () -> Constants.PRESET_INTAKE_INTAKE_POWER
                    ),
                    new HopperHoldCommand(
                        hopperSubsystem,
                        () -> Constants.PRESET_INTAKE_HOPPER_POWER
                    )
                )
            );

        // RT (hold/axis): Manual proportional ground intake + shoot
        operatorController.rightTrigger(Constants.TRIGGER_DEADBAND)
            .whileTrue(
                new ParallelCommandGroup(
                    new LauncherIntakeHoldCommand(
                        launcherSubsystem,
                        () -> MathUtil.applyDeadband(
                            operatorController.getRightTriggerAxis(),
                            Constants.TRIGGER_DEADBAND
                        ) * Constants.MANUAL_GROUND_MAX_LAUNCHER_POWER
                    ),
                    new HopperHoldCommand(
                        hopperSubsystem,
                        () -> MathUtil.applyDeadband(
                            operatorController.getRightTriggerAxis(),
                            Constants.TRIGGER_DEADBAND
                        ) * Constants.MANUAL_GROUND_MAX_HOPPER_POWER
                    )
                )
            );

        // LB (toggle): Auto holding outtake + shoot (launcher + hopper reverse)
        operatorController.leftBumper()
            .toggleOnTrue(
                new ParallelCommandGroup(
                    new LauncherIntakeHoldCommand(
                        launcherSubsystem,
                        () -> Constants.PRESET_OUTTAKE_INTAKE_POWER
                    ),
                    new HopperHoldCommand(
                        hopperSubsystem,
                        () -> Constants.PRESET_OUTTAKE_HOPPER_POWER
                    )
                )
            );

        // LT (hold/axis): Manual proportional holding outtake + shoot
        operatorController.leftTrigger(Constants.TRIGGER_DEADBAND)
            .whileTrue(
                new ParallelCommandGroup(
                    new LauncherIntakeHoldCommand(
                        launcherSubsystem,
                        () -> MathUtil.applyDeadband(
                            operatorController.getLeftTriggerAxis(),
                            Constants.TRIGGER_DEADBAND
                        ) * Constants.MANUAL_HOLDING_MAX_LAUNCHER_POWER
                    ),
                    new HopperHoldCommand(
                        hopperSubsystem,
                        () -> MathUtil.applyDeadband(
                            operatorController.getLeftTriggerAxis(),
                            Constants.TRIGGER_DEADBAND
                        ) * Constants.MANUAL_HOLDING_MAX_HOPPER_POWER
                    )
                )
            );

        // X (hold): Transfer/intake to holding (hopper forward)
        operatorController.x()
            .whileTrue(
                new HopperHoldCommand(
                    hopperSubsystem,
                    () -> Constants.TRANSFER_TO_HOLDING_HOPPER_POWER
                )
            );

        // A (hold): "spins holding in" / retain (launcher low power)
        operatorController.a()
            .whileTrue(
                new LauncherIntakeHoldCommand(
                    launcherSubsystem,
                    () -> Constants.HOLDING_RETAIN_LAUNCHER_POWER
                )
            );

        // Y (hold): spit out / eject (launcher + hopper reverse)
        operatorController.y()
            .whileTrue(
                new ParallelCommandGroup(
                    new LauncherIntakeHoldCommand(
                        launcherSubsystem,
                        () -> Constants.EJECT_LAUNCHER_POWER
                    ),
                    new HopperHoldCommand(
                        hopperSubsystem,
                        () -> Constants.EJECT_HOPPER_POWER
                    )
                )
            );

        // B intentionally unbound
        // operatorController.b() ...
    }

    public Command getAutonomousCommand() {
        return null;
    }
}