package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * IntakeHopperSubsystem
 *
 * Owns the "game piece handling" motors:
 * - Ground intake motor
 * - Hopper / feeder motor
 *
 * Mode-driven: commands request a mode, periodic() applies outputs.
 */
public class IntakeHopperSubsystem extends SubsystemBase {

    public enum Mode {
        STOP,
        TRIGGER_INTAKE,     // manual proportional intake (intake + hopper together)
        TRIGGER_OUTTAKE,    // manual proportional outtake (intake + hopper together, reverse)
        HOPPER_MANUAL,      // hopper fixed power (intake off)
        PRESET_INTAKE,      // intake + hopper together
        PRESET_OUTTAKE,     // reverse intake + hopper
        TRIGGER_LAUNCHERFEED // explicit feed mode (only mode allowed to spin launcher)
    }

    private final WPI_VictorSPX groundIntake;
    private final WPI_VictorSPX hopper;

    private Mode mode = Mode.STOP;

    private double triggerPower = 0.0;
    private double hopperManualPower = 0.0;

    // last applied outputs (what the launcher will “read”)
    private double lastIntakeOut = 0.0;
    private double lastHopperOut = 0.0;

    public IntakeHopperSubsystem(int groundIntakeCanId, int hopperCanId) {
        groundIntake = new WPI_VictorSPX(groundIntakeCanId);
        hopper = new WPI_VictorSPX(hopperCanId);

        groundIntake.setNeutralMode(NeutralMode.Brake);
        hopper.setNeutralMode(NeutralMode.Brake);

        // dashboard labels (nice grouping in Shuffleboard)
        SmartDashboard.putString("IntakeHopper/Mode", mode.name());
        SmartDashboard.putNumber("IntakeHopper/LastIntakeOut", 0.0);
        SmartDashboard.putNumber("IntakeHopper/LastHopperOut", 0.0);
        SmartDashboard.putNumber("IntakeHopper/TriggerPower", 0.0);
        SmartDashboard.putNumber("IntakeHopper/HopperManualPower", 0.0);
    }

    public Mode getMode() {
        return mode;
    }

    public void requestMode(Mode newMode) {
        mode = newMode;
    }

    public void setTriggerPower(double power) {
        triggerPower = power;
    }

    public void setHopperManualPower(double power) {
        hopperManualPower = power;
    }

    public void stop() {
        requestMode(Mode.STOP);
        applyOutputs(0.0, 0.0);
    }

    // getters used by launcher coupling
    public double getLastIntakeOut() {
        return lastIntakeOut;
    }

    public double getLastHopperOut() {
        return lastHopperOut;
    }

    private void applyOutputs(double intakeOut, double hopperOut) {
        lastIntakeOut = intakeOut;
        lastHopperOut = hopperOut;

        groundIntake.set(intakeOut);
        hopper.set(hopperOut);
    }

    @SuppressWarnings("incomplete-switch")
    @Override
    public void periodic() {
        switch (mode) {
            case STOP:
                applyOutputs(0.0, 0.0);
                break;

            case TRIGGER_INTAKE:
                // sketch wants manual intake to drive both intake + hopper together
                applyOutputs(triggerPower, triggerPower);
                break;

            case TRIGGER_OUTTAKE:
                // sketch wants manual outtake to reverse BOTH motors (previously only reversed hopper)
                applyOutputs(-triggerPower, -triggerPower);
                break;

            case HOPPER_MANUAL:
                // sketch wants manual hopper mode to only run hopper; keep intake OFF
                // (previously applied hopperManualPower to BOTH motors)
                applyOutputs(0.0, hopperManualPower);
                break;

            case PRESET_INTAKE:
                applyOutputs(
                    frc.robot.Constants.PRESET_INTAKE_INTAKE_POWER,
                    frc.robot.Constants.PRESET_INTAKE_HOPPER_POWER
                );
                break;

            case PRESET_OUTTAKE:
                applyOutputs(
                    frc.robot.Constants.PRESET_OUTTAKE_INTAKE_POWER,
                    frc.robot.Constants.PRESET_OUTTAKE_HOPPER_POWER
                );
                break;

            case TRIGGER_LAUNCHERFEED:
                // Leave this behavior as-is unless your mechanical direction test proves otherwise.
                applyOutputs(triggerPower, -triggerPower);
                break;
        }

        // SmartDashboard telemetry (lets you validate bindings without a robot present)
        SmartDashboard.putString("IntakeHopper/Mode", mode.name());
        SmartDashboard.putNumber("IntakeHopper/LastIntakeOut", lastIntakeOut);
        SmartDashboard.putNumber("IntakeHopper/LastHopperOut", lastHopperOut);
        SmartDashboard.putNumber("IntakeHopper/TriggerPower", triggerPower);
        SmartDashboard.putNumber("IntakeHopper/HopperManualPower", hopperManualPower);
    }
}