package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
 * - Keep behavior stable now so later Spark Flex changes do NOT alter button mapping
 */
public class RobotContainer {

    // ============================
    // Controllers
    // ============================
    private final XboxController driverController =
        new XboxController(Constants.DRIVER_CONTROLLER_PORT);

    private final XboxController operatorController =
        new XboxController(Constants.OPERATOR_CONTROLLER_PORT);

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
        // - X: intake/transfer to holding
        // - A: spins holding in (hold/retain)
        // - Y: spit out (eject)
        // - B: intentionally left open for now

        // ----------------------------
        // RB (toggle): Auto ground intake + shoot
        // Runs BOTH: launcher motor + hopper feed
        // ----------------------------
        new JoystickButton(operatorController, XboxController.Button.kRightBumper.value)
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

        // ----------------------------
        // RT (hold/axis): Manual proportional ground intake + shoot
        // Proportional scaling: trigger * maxPower
        // ----------------------------
        new Trigger(() -> MathUtil.applyDeadband(
            operatorController.getRightTriggerAxis(),
            Constants.TRIGGER_DEADBAND
        ) > 0.0)
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

        // ----------------------------
        // LB (toggle): Auto holding outtake + shoot
        // Runs BOTH: launcher motor + hopper reverse
        // ----------------------------
        new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value)
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

        // ----------------------------
        // LT (hold/axis): Manual proportional holding outtake + shoot
        // Proportional scaling: trigger * maxPower (negative direction handled by constants)
        // ----------------------------
        new Trigger(() -> MathUtil.applyDeadband(
            operatorController.getLeftTriggerAxis(),
            Constants.TRIGGER_DEADBAND
        ) > 0.0)
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

        // ----------------------------
        // X (hold): Transfer / intake to holding
        // (Simple + reliable: run hopper forward)
        // ----------------------------
        new JoystickButton(operatorController, XboxController.Button.kX.value)
            .whileTrue(
                new HopperHoldCommand(
                    hopperSubsystem,
                    () -> Constants.TRANSFER_TO_HOLDING_HOPPER_POWER
                )
            );

        // ----------------------------
        // A (hold): "Spins holding in" (retain/hold piece)
        // (Simple: run launcher at a low hold power)
        // ----------------------------
        new JoystickButton(operatorController, XboxController.Button.kA.value)
            .whileTrue(
                new LauncherIntakeHoldCommand(
                    launcherSubsystem,
                    () -> Constants.HOLDING_RETAIN_LAUNCHER_POWER
                )
            );

        // ----------------------------
        // Y (hold): Spit out / eject
        // (Run both launcher + hopper reverse)
        // ----------------------------
        new JoystickButton(operatorController, XboxController.Button.kY.value)
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

        // B is intentionally unbound for now.

        // ====================================================
        // DRIVER CONTROLS (Controller 0)
        // ====================================================
        // Driver only drives. (Optional future: add a kill switch here.)
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
