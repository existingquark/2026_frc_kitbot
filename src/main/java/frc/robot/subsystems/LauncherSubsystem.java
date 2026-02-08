package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * LauncherSubsystem
 *
 * Spark Flex (NEO Vortex) implementation.
 * Provides simple percent-output control for commands.
 */
public class LauncherSubsystem extends SubsystemBase {

    private final CANSparkFlex launcherMotor =
        new CANSparkFlex(Constants.LAUNCHER_SPARKFLEX_CAN, MotorType.kBrushless);

    private double lastPercent = 0.0;

    public LauncherSubsystem() {
        launcherMotor.restoreFactoryDefaults();

        launcherMotor.setInverted(Constants.LAUNCHER_INVERTED);
        launcherMotor.setIdleMode(IdleMode.kBrake);

        // Optional: keep CAN usage reasonable
        launcherMotor.setSmartCurrentLimit(60);

        launcherMotor.burnFlash();
    }

    /** Percent output [-1..1]. */
    public void setPercentOutput(double percent) {
        lastPercent = percent;
        launcherMotor.set(percent);
    }

    public void stop() {
        setPercentOutput(0.0);
    }

    public double getLastPercent() {
        return lastPercent;
    }
}
