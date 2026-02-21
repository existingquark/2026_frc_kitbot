package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

// QuestNav imports
import frc.robot.subsystems.QuestNavSubsystem.QuestNavConstants;
import frc.robot.subsystems.QuestNavSubsystem.QuestNavHealth;
import frc.robot.subsystems.QuestNavSubsystem.QuestNavPose;
import frc.robot.subsystems.QuestNavSubsystem.QuestNavSubsystem;

/**
 * DriveSubsystem
 *
 * Purpose:
 * - Owns and controls the robot drivetrain hardware.
 * - Provides a simple arcade-drive interface for commands to use.
 * - Owns the robot pose estimator (single source of truth for pose).
 *
 * How it fits in the robot:
 * - Created by RobotContainer.
 * - Used by a default drive command (e.g., ArcadeDriveCommand).
 * - Must be commanded every scheduler loop to satisfy MotorSafety.
 *
 * Key assumptions / constraints:
 * - Tank-style drivetrain with two motors per side.
 * - CAN IDs are based on the original time-based robot code.
 *
 * QuestNav integration rules:
 * - QuestNav does NOT drive motors.
 * - QuestNav provides an external pose measurement (treated like vision).
 * - DriveSubsystem decides whether to accept/reject Quest pose data.
 *
 * Future expansion hooks:
 * - Integrate encoders and gyro
 * - Update estimator every loop with encoder+gyro
 * - Fuse QuestNav pose into estimator (already scaffolded here)
 */
@SuppressWarnings("removal")
public class DriveSubsystem extends SubsystemBase {

    // ============================
    // Drivetrain Motors (CAN)
    // ============================
    private final WPI_VictorSPX leftFront = new WPI_VictorSPX(frc.robot.Constants.LEFT_FRONT_CAN);
    private final WPI_VictorSPX leftBack  = new WPI_VictorSPX(frc.robot.Constants.LEFT_BACK_CAN);

    private final WPI_VictorSPX rightFront = new WPI_VictorSPX(frc.robot.Constants.RIGHT_FRONT_CAN);
    private final WPI_VictorSPX rightBack  = new WPI_VictorSPX(frc.robot.Constants.RIGHT_BACK_CAN);

    // ============================
    // Motor Groups
    // ============================
    private final MotorControllerGroup leftSide =
        new MotorControllerGroup(leftFront, leftBack);

    private final MotorControllerGroup rightSide =
        new MotorControllerGroup(rightFront, rightBack);

    // WPILib helper class that implements arcade/tank drive math
    private final DifferentialDrive drive =
        new DifferentialDrive(leftSide, rightSide);

    // ============================
    // QuestNav (external pose source)
    // ============================
    // Mentor note:
    // - This subsystem reads NT4 and exposes pose + timing + health.
    // - DriveSubsystem is responsible for fusing pose into its estimator.
    private final QuestNavSubsystem questNav = new QuestNavSubsystem();

    // Mentor note:
    // - Track last accepted pose to reject teleports/spikes.
    private Optional<Pose2d> lastAcceptedQuest = Optional.empty();

    // ============================
    // Pose Estimator (scaffold)
    // ============================
    // Mentor note:
    // - This becomes the robot's single source of truth for pose.
    // - It fuses:
    //   (A) drivetrain sensors (encoders + gyro) and
    //   (B) external pose measurements (QuestNav, treated like vision)
    //
    // IMPORTANT TODO:
    // - Set track width to match kitbot build. Typical range ~0.55-0.65m.
    private static final double TRACK_WIDTH_M = 0.60;

    private final DifferentialDriveKinematics kinematics =
        new DifferentialDriveKinematics(TRACK_WIDTH_M);

    // Mentor note:
    // - These are starting noise values; tune once sensors are real.
    private final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, 0.02);
    private final Matrix<N3, N1> localMeasurementStdDevs = VecBuilder.fill(0.02, 0.02, 0.01);
    private final Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.20, 0.20, 0.10);


    private final DifferentialDrivePoseEstimator estimator =
        new DifferentialDrivePoseEstimator(
            kinematics,
            getHeading(),
            getLeftDistanceMeters(),
            getRightDistanceMeters(),
            new Pose2d(),
            stateStdDevs,
            localMeasurementStdDevs
        );

    public DriveSubsystem() {

        // ============================
        // Neutral Mode Configuration
        // ============================
        leftFront.setNeutralMode(NeutralMode.Brake);
        leftBack.setNeutralMode(NeutralMode.Brake);
        rightFront.setNeutralMode(NeutralMode.Brake);
        rightBack.setNeutralMode(NeutralMode.Brake);

        // ============================
        // Motor Inversion
        // ============================
        // Mentor note:
        // - Inversion should be centralized so it’s easy to change wiring direction.
        rightSide.setInverted(frc.robot.Constants.RIGHT_SIDE_INVERTED);

        // Mentor note:
        // - WPILib 2026 uses a setter for vision measurement standard deviations.
        estimator.setVisionMeasurementStdDevs(visionStdDevs);
    }

    /**
     * arcadeDrive
     *
     * Drives the robot using arcade-style controls.
     *
     * Inputs:
     * - forward: forward/backward command (-1.0 to +1.0)
     * - turn: rotation command (-1.0 to +1.0)
     */
    public void arcadeDrive(double forward, double turn) {
        double leftPower = forward + turn;
        double rightPower = forward - turn;

        leftPower = MathUtil.clamp(leftPower, -1.0, 1.0) * frc.robot.Constants.LEFT_DRIVETRAIN_MAX_POWER;
        rightPower = MathUtil.clamp(rightPower, -1.0, 1.0) * frc.robot.Constants.RIGHT_DRIVETRAIN_MAX_POWER;

        drive.tankDrive(leftPower, rightPower);
    }

    /** Stops the drivetrain immediately. */
    public void stop() {
        drive.arcadeDrive(0.0, 0.0);
    }

    // ============================
    // Pose Accessors (for auto / commands)
    // ============================
    // Mentor note:
    // - Commands should consume pose from DriveSubsystem, not from QuestNav directly.
    public Pose2d getEstimatedPose() {
        return estimator.getEstimatedPosition();
    }

    // Mentor note:
    // - Reset estimator pose at key times (auto init, test, etc.)
    // - With real sensors, this must also reset encoder baselines as needed.
    public void resetEstimatedPose(Pose2d pose) {
        estimator.resetPosition(getHeading(), getLeftDistanceMeters(), getRightDistanceMeters(), pose);
        lastAcceptedQuest = Optional.empty();
    }

    @Override
    public void periodic() {
        // ============================
        // QuestNav update
        // ============================
        // Mentor note:
        // - Pull latest NT4 state.
        // - QuestNavSubsystem also publishes its own dashboards/Field2d.
        questNav.periodic();

        // ============================
        // Estimator update (drivetrain sensors)
        // ============================
        // Mentor note:
        // - Once encoders + gyro are wired, these values will move continuously.
        // - Until then, they are stubs and estimator relies mainly on Quest corrections.
        estimator.update(getHeading(), getLeftDistanceMeters(), getRightDistanceMeters());

        // ============================
        // QuestNav fusion (vision-like)
        // ============================
        // Mentor note:
        // - Gate by connection, staleness, quality, and teleport checks.
        // - Only accepted samples are fused. 
        fuseQuestIfValid();

        // ============================
        // Debug telemetry
        // ============================
        Pose2d est = estimator.getEstimatedPosition();
        SmartDashboard.putNumber("PoseEst/X_m", est.getX());
        SmartDashboard.putNumber("PoseEst/Y_m", est.getY());
        SmartDashboard.putNumber("PoseEst/Yaw_deg", est.getRotation().getDegrees());
    }

    private void fuseQuestIfValid() {
        QuestNavHealth health = questNav.getHealth();
        Optional<QuestNavPose> poseOpt = questNav.getLatestPoseSample();
        Optional<Double> rioTsOpt = questNav.getLatestRioTimestampSeconds();

        if (poseOpt.isEmpty() || rioTsOpt.isEmpty()) return;

        QuestNavPose s = poseOpt.get();
        Pose2d questPose = s.toPose2d();
        double measRioTs = rioTsOpt.get();

        if (!s.connected) return;
        if (health.stale) return;
        if (s.quality < QuestNavConstants.MIN_QUALITY) return;

        if (lastAcceptedQuest.isPresent()) {
            Pose2d last = lastAcceptedQuest.get();
            double dx = questPose.getX() - last.getX();
            double dy = questPose.getY() - last.getY();
            double dist = Math.hypot(dx, dy);

            double dyaw = questPose.getRotation().minus(last.getRotation()).getRadians();
            dyaw = Math.atan2(Math.sin(dyaw), Math.cos(dyaw));

            if (dist > QuestNavConstants.MAX_POSITION_JUMP_M) return;
            if (Math.abs(dyaw) > QuestNavConstants.MAX_YAW_JUMP_RAD) return;
        }

        estimator.addVisionMeasurement(questPose, measRioTs);
        lastAcceptedQuest = Optional.of(questPose);
        SmartDashboard.putBoolean("Quest/FusedThisLoop", true);
    }

    // ============================
    // Sensor hooks (must implement)
    // ============================
    // Mentor note:
    // - Estimator correctness depends on these:
    //   (A) gyro heading
    //   (B) left & right wheel distances (meters)
    //
    // - Until sensors exist, these stubs return 0 and estimator will not dead-reckon.
    // - QuestNav fusion can still function for pipeline testing.

    private Rotation2d getHeading() {
        return new Rotation2d();
    }

    private double getLeftDistanceMeters() {
        return 0.0;
    }

    private double getRightDistanceMeters() {
        return 0.0;
    }
}
