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
 * This subsystem is mode-driven: commands request an intent (mode),
 * and the subsystem applies the correct motor outputs in periodic().
 * This keeps behavior deterministic and easy to extend for autonomous.
 */
public class IntakeHopperSubsystem extends SubsystemBase {

    /**
     * Operating modes for the intake/hopper cluster.
     *
     * Modes encode safe "intent" so commands do not fight over outputs.
     * This is also friendly to autonomous, which can schedule the same commands.
     */
    public enum Mode {
        STOP,
        TRIGGER_INTAKE,    // intake runs, hopper off (safety)
        TRIGGER_OUTTAKE,   // hopper runs, intake off (safety)
        HOPPER_MANUAL,     // hopper fixed power
        PRESET_INTAKE,     // intake + hopper together
        PRESET_OUTTAKE,    // reverse intake + hopper
        LAUNCHER_HOLD      // intake runs continuously while enabled
    }

    // Motor controller for pulling game pieces in from the floor
    private final WPI_VictorSPX groundIntake;

    // Motor controller for moving game pieces through the hopper/feeder
    private final WPI_VictorSPX hopper;

    // Current requested mode (set by commands)
    private Mode mode = Mode.STOP;

    // Parameter used by trigger modes
    private double triggerPower = 0.0;

    // Parameter used by manual hopper mode
    private double hopperManualPower = 0.0;

    /**
     * Constructs the subsystem and applies basic motor configuration.
     *
     * @param groundIntakeCanId CAN ID for the ground intake VictorSPX
     * @param hopperCanId       CAN ID for the hopper VictorSPX
     */
    public IntakeHopperSubsystem(int groundIntakeCanId, int hopperCanId) {
        groundIntake = new WPI_VictorSPX(groundIntakeCanId);
        hopper = new WPI_VictorSPX(hopperCanId);

        // Brake mode helps the mechanism stop quickly when output goes to zero
        groundIntake.setNeutralMode(NeutralMode.Brake);
        hopper.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Returns the currently requested operating mode.
     *
     * @return current mode
     */
    public Mode getMode() {
        return mode;
    }

    /**
     * Requests a new operating mode.
     *
     * Commands should call this to declare intent instead of setting motors directly.
     *
     * @param newMode desired mode
     */
    public void requestMode(Mode newMode) {
        mode = newMode;
    }

    /**
     * Sets the power parameter used by trigger-controlled modes.
     *
     * @param power trigger axis power (typically 0.0 to 1.0)
     */
    public void setTriggerPower(double power) {
        triggerPower = power;
    }

    /**
     * Sets the power parameter used by manual hopper mode.
     *
     * @param power hopper output (negative reverses)
     */
    public void setHopperManualPower(double power) {
        hopperManualPower = power;
    }

    /**
     * Stops motors and returns to STOP mode.
     *
     * This is a safe default called when commands end or are interrupted.
     */
    public void stop() {
        requestMode(Mode.STOP);
        applyOutputs(0.0, 0.0);
    }

    /**
     * Applies raw motor outputs.
     *
     * Keeping output assignment here makes it easy to add logging,
     * output limiting, or safety checks later.
     *
     * @param intakeOut output for ground intake motor
     * @param hopperOut output for hopper motor
     */
    private void applyOutputs(double intakeOut, double hopperOut) {
        groundIntake.set(intakeOut);
        hopper.set(hopperOut);
    }

    /**
     * Periodic subsystem update.
     *
     * This is the single authoritative translation from requested mode → motor outputs.
     * That makes behavior predictable and reduces command conflicts.
     */
    @SuppressWarnings("incomplete-switch")
    @Override
    public void periodic() {

        // Translate the requested mode into the correct motor outputs
        switch (mode) {

            case STOP:
                applyOutputs(0.0, 0.0);
                break;

            case TRIGGER_INTAKE:
                // Safety logic: intake runs, hopper is forced off
                applyOutputs(triggerPower, 0.0);
                break;

            case TRIGGER_OUTTAKE:
                // Safety logic: hopper runs, intake is forced off
                applyOutputs(0.0, triggerPower);
                break;

            case HOPPER_MANUAL:
                applyOutputs(0.0, hopperManualPower);
                break;

            case PRESET_INTAKE:
                applyOutputs(frc.robot.Constants.PRESET_INTAKE_INTAKE_POWER, frc.robot.Constants.PRESET_INTAKE_HOPPER_POWER);
                break;

            case PRESET_OUTTAKE:
                applyOutputs(frc.robot.Constants.PRESET_OUTTAKE_INTAKE_POWER, frc.robot.Constants.PRESET_OUTTAKE_HOPPER_POWER);
                break;

            // case LAUNCHER_HOLD:
            //     // Mirrors old behavior: when launcher is "on", intake holds at 0.8
            //     applyOutputs(frc.robot.Constants.LAUNCHER_HOLD_INTAKE_POWER, 0.0);
            //     break;
        }
    }
}