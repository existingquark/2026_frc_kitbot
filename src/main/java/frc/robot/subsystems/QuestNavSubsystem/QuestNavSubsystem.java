package frc.robot.subsystems.QuestNavSubsystem;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
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
 * Data sources:
 * - NT4 stream from Jetson (Quest pose -> /questnav keys)
 *
 * Consumers:
 * - Driver assist commands
 * - Autonomous commands
 * - DriveSubsystem pose estimator fusion
 */
public class QuestNavSubsystem extends SubsystemBase {

    // ============================
    // IO (NT4)
    // ============================
    private final QuestNavIO io;

    // ============================
    // Navigation state (for commands)
    // ============================
    // Current robot pose (field-relative)
    private Pose2d currentPose = new Pose2d();

    // Targeting data (relative to robot)
    private double distanceToTargetMeters = 0.0;
    private double headingErrorDegrees = 0.0;

    // ============================
    // Dashboard visualization
    // ============================
    private final Field2d field = new Field2d();

    public QuestNavSubsystem() {
        this(new QuestNavIONT4());
    }

    public QuestNavSubsystem(QuestNavIO io) {
        this.io = io;
        SmartDashboard.putData("QuestNav Field", field);
    }

    @Override
    public void periodic() {
        // Pull the latest NT4 sample
        io.periodic();

        // If we have a valid pose sample, update our stored pose
        Optional<QuestNavPose> sample = io.getLatest();
        if (sample.isPresent()) {
            currentPose = sample.get().toPose2d();
        }

        // Always publish pose to Field2d
        field.setRobotPose(currentPose);

        // Health + debug outputs
        QuestNavHealth health = io.getHealth();
        SmartDashboard.putBoolean("Quest/Connected", health.connected);
        SmartDashboard.putNumber("Quest/Age_s", health.ageSeconds);
        SmartDashboard.putBoolean("Quest/Stale", health.stale);
        SmartDashboard.putNumber("Quest/Quality", health.quality);

        SmartDashboard.putNumber("QuestNav/X (m)", currentPose.getX());
        SmartDashboard.putNumber("QuestNav/Y (m)", currentPose.getY());
        SmartDashboard.putNumber("QuestNav/Heading (deg)", currentPose.getRotation().getDegrees());
        SmartDashboard.putNumber("QuestNav/Target Distance (m)", distanceToTargetMeters);
        SmartDashboard.putNumber("QuestNav/Heading Error (deg)", headingErrorDegrees);
    }

    // ============================
    // Pose update interface (manual override)
    // ============================

    /**
     * Manual override pose. Useful for testing or if you temporarily want to drive pose
     * from another source. Note: periodic() will overwrite this when NT has data.
     */
    public void updatePose(Pose2d newPose) {
        currentPose = newPose;
    }

    public void resetPose() {
        currentPose = new Pose2d();
    }

    // ============================
    // Targeting update interface
    // ============================

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

    public DoubleSupplier getDistanceToTargetSupplier() {
        return () -> distanceToTargetMeters;
    }

    public DoubleSupplier getHeadingErrorSupplier() {
        return () -> headingErrorDegrees;
    }

    public DoubleSupplier getHeadingSupplier() {
        return () -> currentPose.getRotation().getDegrees();
    }

    // ============================
    // Accessors (for DriveSubsystem fusion)
    // ============================

    /** Latest raw QuestNav sample (may be stale). */
    public Optional<QuestNavPose> getLatestPoseSample() {
        return io.getLatest();
    }

    /** Converted timestamp in roboRIO time base (for addVisionMeasurement). */
    public Optional<Double> getLatestRioTimestampSeconds() {
        return io.getLatestRioTimestampSeconds();
    }

    public QuestNavHealth getHealth() {
        return io.getHealth();
    }
}
