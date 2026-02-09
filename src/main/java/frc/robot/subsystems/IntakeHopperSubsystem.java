package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

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
        TRIGGER_INTAKE,    // intake runs, hopper off (safety)
        TRIGGER_OUTTAKE,   // hopper runs, intake off (safety)
        HOPPER_MANUAL,     // hopper fixed power
        PRESET_INTAKE,     // intake + hopper together
        PRESET_OUTTAKE     // reverse intake + hopper
    }

    private final WPI_VictorSPX groundIntake;
    private final WPI_VictorSPX hopper;

    private Mode mode = Mode.STOP;

    private double triggerPower = 0.0;
    private double hopperManualPower = 0.0;

    // ✅ NEW: last applied outputs (what the launcher will “read”)
    private double lastIntakeOut = 0.0;
    private double lastHopperOut = 0.0;

    public IntakeHopperSubsystem(int groundIntakeCanId, int hopperCanId) {
        groundIntake = new WPI_VictorSPX(groundIntakeCanId);
        hopper = new WPI_VictorSPX(hopperCanId);

        groundIntake.setNeutralMode(NeutralMode.Brake);
        hopper.setNeutralMode(NeutralMode.Brake);
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

    // ✅ NEW: getters used by launcher coupling
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
                applyOutputs(triggerPower, 0.0);
                break;

            case TRIGGER_OUTTAKE:
                applyOutputs(0.0, triggerPower);
                break;

            case HOPPER_MANUAL:
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
        }
    }
}
