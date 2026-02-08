package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * QuestNavSubsystem
 *
 * Purpose:
 * - Central navigation and targeting interface for the robot.
 *
 * Design rules (mentor notes):
 * - QuestNav does NOT control motors.
 * - QuestNav does NOT own drivetrain hardware.
 * - QuestNav stores and publishes navigation state (pose, heading error, distance).
 *
 * Data sources (now or future):
 * - Meta Quest 3 pose tracking
 * - Vision targeting
 * - Autonomous scripts
 *
 * Consumers:
 * - Driver assist commands
 * - Autonomous commands
 */
public class QuestNavSubsystem extends SubsystemBase {

    // ============================
    // Navigation state
    // ============================
    // Current estimated robot pose on the field (meters, radians)
    private Pose2d currentPose = new Pose2d();

    // Targeting data (relative to robot)
    private double distanceToTargetMeters = 0.0;
    private double headingErrorDegrees = 0.0;

    // ============================
    // Dashboard visualization
    // ============================
    private final Field2d field = new Field2d();

    public QuestNavSubsystem() {
        SmartDashboard.putData("QuestNav Field", field);
    }

    @Override
    public void periodic() {
        // Always publish pose to the dashboard for visualization
        field.setRobotPose(currentPose);

        // Optional debug outputs (very helpful early on)
        SmartDashboard.putNumber("QuestNav/X (m)", currentPose.getX());
        SmartDashboard.putNumber("QuestNav/Y (m)", currentPose.getY());
        SmartDashboard.putNumber(
            "QuestNav/Heading (deg)",
            currentPose.getRotation().getDegrees()
        );
        SmartDashboard.putNumber("QuestNav/Target Distance (m)", distanceToTargetMeters);
        SmartDashboard.putNumber("QuestNav/Heading Error (deg)", headingErrorDegrees);
    }

    // ============================
    // Pose update interface (Quest 3)
    // ============================

    /**
     * Update the robot's pose using Quest-based tracking.
     *
     * Mentor note:
     * - This should be called by the Quest integration layer.
     * - Pose should already be expressed in meters and field-relative.
     */
    public void updatePose(Pose2d newPose) {
        currentPose = newPose;
    }

    /**
     * Convenience overload for raw values.
     *
     * @param xMeters robot X position (meters)
     * @param yMeters robot Y position (meters)
     * @param headingDegrees robot heading (degrees, CCW positive)
     */
    public void updatePose(double xMeters, double yMeters, double headingDegrees) {
        currentPose = new Pose2d(
            xMeters,
            yMeters,
            Rotation2d.fromDegrees(headingDegrees)
        );
    }

    /**
     * Reset pose (recommended at auto init).
     *
     * Mentor note:
     * - Always reset Quest pose at the start of autonomous.
     * - This defines the coordinate frame for the entire auto routine.
     */
    public void resetPose() {
        currentPose = new Pose2d();
    }

    // ============================
    // Targeting update interface
    // ============================

    /**
     * Update targeting data (vision or Quest-derived).
     *
     * @param distanceMeters distance to target
     * @param headingErrorDeg signed heading error to target
     */
    public void updateTargeting(double distanceMeters, double headingErrorDeg) {
        distanceToTargetMeters = distanceMeters;
        headingErrorDegrees = headingErrorDeg;
    }

    // ============================
    // Accessors (for commands)
    // ============================

    public Pose2d getCurrentPose() {
        return currentPose;
    }

    /**
     * Supplier for distance-to-target.
     *
     * Used by autonomous approach and scoring commands.
     */
    public DoubleSupplier getDistanceToTargetSupplier() {
        return () -> distanceToTargetMeters;
    }

    /**
     * Supplier for heading error.
     *
     * Used by auto-align and driver assist commands.
     */
    public DoubleSupplier getHeadingErrorSupplier() {
        return () -> headingErrorDegrees;
    }

    /**
     * Supplier for robot heading (degrees).
     *
     * Useful for turn-to-angle commands.
     */
    public DoubleSupplier getHeadingSupplier() {
        return () -> currentPose.getRotation().getDegrees();
    }
}
