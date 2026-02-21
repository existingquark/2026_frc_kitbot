package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * HopperSubsystem
 *
 * Purpose:
 * - Owns the hopper motor hardware and provides simple methods to move game pieces
 *   toward or away from the launcher path.
 *
 * How it fits in the robot:
 * - Created by RobotContainer.
 * - Controlled by commands that define when/how the hopper runs.
 *
 * Key assumptions / constraints:
 * - CAN ID 1 controls the hopper motor (verified in Phoenix Tuner X).
 *
 * Future expansion hooks:
 * - Add beam break sensor input for automatic feed control.
 * - Add autonomous feed sequencing once launcher velocity control is implemented.
 */
public class HopperSubsystem extends SubsystemBase {

    // ============================
    // Hardware: Hopper Motor (CAN)
    // ============================
    private final WPI_VictorSPX hopperMotor = new WPI_VictorSPX(frc.robot.Constants.HOPPER_CAN);

    /**
     * Constructor
     *
     * Applies one-time motor configuration.
     */
    public HopperSubsystem() {
        hopperMotor.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * setPercentOutput
     *
     * Runs the hopper motor using open-loop percent output.
     *
     * Inputs:
     * - percent: [-1..1] (positive direction defined by mechanism testing).
     */
    public void setPercentOutput(double percent) {
        hopperMotor.set(percent);
    }

    /**
     * stop
     *
     * Stops the hopper motor.
     */
    public void stop() {
        hopperMotor.set(0.0);
    }
}
