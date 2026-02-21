package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.FeedCoupledLauncherCommand;
import frc.robot.commands.LauncherIntakeHoldCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeHopperSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class RobotContainer {

    private final CommandXboxController driverController =
        new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);

    private final CommandXboxController operatorController =
        new CommandXboxController(Constants.OPERATOR_CONTROLLER_PORT);

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();

    private final IntakeHopperSubsystem intakeHopperSubsystem =
        new IntakeHopperSubsystem(Constants.INTAKE_CAN, Constants.HOPPER_CAN);

    private final LauncherSubsystem launcherSubsystem = new LauncherSubsystem();

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
    }

    private void configureDefaultCommands() {
        driveSubsystem.setDefaultCommand(
            new ArcadeDriveCommand(driveSubsystem, driverController)
        );

        // ✅ Launcher follows feed automatically
        // "follow" is now gated so launcher stays OFF unless explicitly in LAUNCHERFEED mode
        launcherSubsystem.setDefaultCommand(
            new FeedCoupledLauncherCommand(launcherSubsystem, intakeHopperSubsystem)
        );
    }

    private void configureButtonBindings() {

        // RB (toggle): auto ground intake (intake + hopper preset)
        operatorController.rightBumper()
            .toggleOnTrue(
                new StartEndCommand(
                    () -> intakeHopperSubsystem.requestMode(IntakeHopperSubsystem.Mode.PRESET_INTAKE),
                    intakeHopperSubsystem::stop,
                    intakeHopperSubsystem
                )
            );

        // RT (hold): manual proportional intake (TRIGGER_INTAKE)
        operatorController.rightTrigger(Constants.TRIGGER_DEADBAND)
            .whileTrue(
                new RunCommand(
                    () -> {
                        double t = MathUtil.applyDeadband(
                            operatorController.getRightTriggerAxis(),
                            Constants.TRIGGER_DEADBAND
                        );
                        intakeHopperSubsystem.requestMode(IntakeHopperSubsystem.Mode.TRIGGER_INTAKE);
                        intakeHopperSubsystem.setTriggerPower(t);
                    },
                    intakeHopperSubsystem
                ).finallyDo(interrupted -> intakeHopperSubsystem.stop())
            );

        // LB (toggle): auto outtake (preset outtake)
        operatorController.leftBumper()
            .toggleOnTrue(
                new StartEndCommand(
                    () -> intakeHopperSubsystem.requestMode(IntakeHopperSubsystem.Mode.PRESET_OUTTAKE),
                    intakeHopperSubsystem::stop,
                    intakeHopperSubsystem
                )
            );

        // LT (hold): manual proportional outtake (TRIGGER_OUTTAKE)
        operatorController.leftTrigger(Constants.TRIGGER_DEADBAND)
            .whileTrue(
                new RunCommand(
                    () -> {
                        double t = MathUtil.applyDeadband(
                            operatorController.getLeftTriggerAxis(),
                            Constants.TRIGGER_DEADBAND
                        );
                        intakeHopperSubsystem.requestMode(IntakeHopperSubsystem.Mode.TRIGGER_OUTTAKE);
                        intakeHopperSubsystem.setTriggerPower(t);
                    },
                    intakeHopperSubsystem
                ).finallyDo(interrupted -> intakeHopperSubsystem.stop())
            );

        // X: Prime Launcher (preset intake)
        operatorController.x()
            .whileTrue(
                new StartEndCommand(
                    () -> {
                        // Prime using preset intake profile (intake + hopper)
                        intakeHopperSubsystem.requestMode(IntakeHopperSubsystem.Mode.PRESET_INTAKE);
                    },
                    intakeHopperSubsystem::stop,
                    intakeHopperSubsystem
                )
            );

        // A (hold) : retain / hold piece (explicit launcher override)
        operatorController.a()
            .whileTrue(
                new LauncherIntakeHoldCommand(
                    launcherSubsystem,
                    () -> Constants.HOLDING_RETAIN_LAUNCHER_POWER
                )
            );

        // Y (hold): eject (explicit launcher override)
        operatorController.y()
            .whileTrue(
                new RunCommand(
                    () -> {
                        double t = 0.75;
                        intakeHopperSubsystem.requestMode(IntakeHopperSubsystem.Mode.TRIGGER_OUTTAKE);
                        intakeHopperSubsystem.setTriggerPower(t);
                    },
                    intakeHopperSubsystem
                ).finallyDo(interrupted -> intakeHopperSubsystem.stop())
            );
    //     // B (hold): smoke test (explicit launcher override)
    //     operatorController.b()
    //         .whileTrue(
    //             new LauncherIntakeHoldCommand(
    //                 launcherSubsystem,
    //                 () -> 0.18
    //             )
    //         );
    }

    public Command getAutonomousCommand() {
        return null;
    }
}