package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * LauncherSubsystem
 *
 * Spark Flex (NEO Vortex) implementation using REVLib 2025+ API.
 * Keeps a stable "percent output" interface for commands.
 */
public class LauncherSubsystem extends SubsystemBase {

    private final SparkFlex launcherMotor =
        new SparkFlex(Constants.LAUNCHER_SPARKFLEX_CAN, MotorType.kBrushless);

    private double lastPercent = 0.0;

    public LauncherSubsystem() {
        // REVLib 2025+ uses config objects + configure()
        SparkFlexConfig config = new SparkFlexConfig();

        config
            .inverted(Constants.LAUNCHER_INVERTED)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.LAUNCHER_CURRENT_LIMIT_AMPS);

        // Safe default: reset safe parameters, and persist to flash
        launcherMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Percent output [-1.0, 1.0]. */
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
