package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
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
 * Central wiring point for:
 * - Subsystems
 * - Commands
 * - Controller bindings
 *
 * Mentor design goals:
 * - Clear separation between DRIVER and OPERATOR roles
 * - Demo-safe, predictable behavior
 * - Easy to swap control ownership without refactoring
 */
public class RobotContainer {

    // ============================
    // Controllers
    // ============================
    // Driver: robot motion + ground intake (current demo config)
    // Operator: launcher only
    private final XboxController driverController =
        new XboxController(0);

    private final XboxController operatorController =
        new XboxController(1);

    // ============================
    // Subsystems
    // ============================
    private final DriveSubsystem driveSubsystem =
        new DriveSubsystem();

    private final HopperSubsystem hopperSubsystem =
        new HopperSubsystem();

    private final LauncherSubsystem launcherSubsystem =
        new LauncherSubsystem();

    // ============================
    // Constructor
    // ============================
    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
    }

    // ============================
    // Default Commands
    // ============================
    /**
     * Default commands
     *
     * Design notes:
     * - Default commands run whenever no other command
     *   requires the subsystem.
     * - Drive should almost always have a default command.
     * - Driver control layout lives inside ArcadeDriveCommand.
     */
    private void configureDefaultCommands() {
        driveSubsystem.setDefaultCommand(
            new ArcadeDriveCommand(
                driveSubsystem,
                driverController
            )
        );
    }

    // ============================
    // Button Bindings
    // ============================
    /**
     * Button bindings
     *
     * CURRENT DEMO CONFIGURATION:
     * - DRIVER controls ground intake / hopper
     * - OPERATOR controls launcher
     *
     * IMPORTANT:
     * - Ground intake ownership can be swapped later by
     *   moving ONE block of bindings (see notes below).
     */
    private void configureButtonBindings() {

        // ====================================================
        // OPERATOR CONTROLS
        // ====================================================
        // Launcher-only controls for the demo.
        // This keeps operator responsibility focused and clear.
        // ====================================================

        // ============================
        // Launcher Intake: Automatic Toggle (RB)
        // ============================
        // Repurposed operator RB to toggle automatic intake mode.
        new JoystickButton(
            operatorController,
            XboxController.Button.kRightBumper.value
        ).toggleOnTrue(
            new LauncherIntakeHoldCommand(
                launcherSubsystem,
                () -> Constants.INTAKE_POWER
            )
        );

        // ============================
        // Launcher Intake / Outtake (X / Y): 
        // ============================
        // These require LauncherSubsystem, so they automatically
    // interlock with LauncherHoldCommand.
        new JoystickButton(
            operatorController,
            XboxController.Button.kX.value
        ).whileTrue(
            new LauncherIntakeHoldCommand(
                launcherSubsystem,
                () -> Constants.INTAKE_POWER
            )
        );

        new JoystickButton(
            operatorController,
            XboxController.Button.kY.value
        ).whileTrue(
            new LauncherIntakeHoldCommand(
                launcherSubsystem,
                () -> Constants.OUTTAKE_POWER
            )
        );

        // ============================
        // Operator Triggers: Manual Proportional Intake/Outtake
        // ============================
        // Right trigger: manual proportional intake
        new Trigger(() -> MathUtil.applyDeadband(
            operatorController.getRightTriggerAxis(),
            Constants.TRIGGER_DEADBAND
        ) > 0.0).whileTrue(
            new LauncherIntakeHoldCommand(
                launcherSubsystem,
                () -> MathUtil.applyDeadband(
                    operatorController.getRightTriggerAxis(),
                    Constants.TRIGGER_DEADBAND
                )
            )
        );

        // Left trigger: manual proportional outtake (negated)
        new Trigger(() -> MathUtil.applyDeadband(
            operatorController.getLeftTriggerAxis(),
            Constants.TRIGGER_DEADBAND
        ) > 0.0).whileTrue(
            new LauncherIntakeHoldCommand(
                launcherSubsystem,
                () -> -MathUtil.applyDeadband(
                    operatorController.getLeftTriggerAxis(),
                    Constants.TRIGGER_DEADBAND
                )
            )
        );

        // ====================================================
        // DRIVER CONTROLS
        // ====================================================
        /*
         * DRIVER CONTROLS (DEMO CONFIGURATION)
         *
         * - Driver controls robot motion (ArcadeDriveCommand)
         * - Driver also controls ground intake / hopper
         *
         * RATIONALE:
         * - Immediate response during demos
         * - Fewer coordination errors
         * - Simpler explanation to parents
         */

        // ============================
        // Ground Intake / Hopper (A / B): Hold-to-Run
        // ============================
        // A: intake forward
        // B: intake reverse (unjam)
        // Move ground intake control to the OPERATOR (requested)
        new JoystickButton(
            operatorController,
            XboxController.Button.kA.value
        ).whileTrue(
            new HopperHoldCommand(
                hopperSubsystem,
                () -> Constants.HOPPER_FORWARD_POWER
            )
        );

        new JoystickButton(
            operatorController,
            XboxController.Button.kB.value
        ).whileTrue(
            new HopperHoldCommand(
                hopperSubsystem,
                () -> Constants.HOPPER_REVERSE_POWER
            )
        );

    // ====================================================
    // FUTURE SWAP POINT (DO NOT ENABLE FOR DEMO)
    // ====================================================
        /*
         * To move ground intake control to the OPERATOR:
         *
         * 1) COMMENT OUT the two bindings above (driver A/B)
         * 2) UNCOMMENT the two bindings below
         * 3) No other code changes are required
         */

        /*
        new JoystickButton(
            operatorController,
            XboxController.Button.kA.value
        ).whileTrue(
            new HopperHoldCommand(
                hopperSubsystem,
                () -> Constants.HOPPER_FORWARD_POWER
            )
        );

        new JoystickButton(
            operatorController,
            XboxController.Button.kB.value
        ).whileTrue(
            new HopperHoldCommand(
                hopperSubsystem,
                () -> Constants.HOPPER_REVERSE_POWER
            )
        );
        */
    }

    // ============================
    // Autonomous (not used for demo)
    // ============================
    public Command getAutonomousCommand() {
        return null;
    }
}
