package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * DriveSubsystem
 *
 * Purpose:
 * - Owns and controls the robot drivetrain hardware.
 * - Provides a simple arcade-drive interface for commands to use.
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
 * Future expansion hooks:
 * - Integrate encoders and gyro
 * - Update odometry every loop
 * - Provide pose to QuestNav for navigation/autonomous
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

    // // ============================
    // // Odometry (scaffold)
    // // ============================
    // // Mentor note:
    // // - Odometry estimates the robot pose (x, y, heading) on the field.
    // // - It normally uses encoder distances + a gyro angle.
    // // - We are scaffolding this now so QuestNav can consume pose cleanly.
    // private final DifferentialDriveOdometry odometry =
    //     new DifferentialDriveOdometry(new Rotation2d());

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
        drive.arcadeDrive(forward, turn);
    }

    /** Stops the drivetrain immediately. */
    public void stop() {
        drive.arcadeDrive(0.0, 0.0);
    }
    // /**
    //  * getPose
    //  *
    //  * Returns the robot's estimated field pose.
    //  *
    //  * Mentor note:
    //  * - This is the "single source of truth" for pose in the drivetrain.
    //  * - QuestNav should consume this (not recreate odometry elsewhere).
    //  */
    // public Pose2d getPose() {
    //     return odometry.getPoseMeters();
    // }

    // /**
    //  * resetPose
    //  *
    //  * Allows autonomous / testing code to reset the robot's pose estimate.
    //  *
    //  * Mentor note:
    //  * - When you add gyro + encoders, this should also reset sensor baselines.
    //  */
    // public void resetPose(Pose2d newPose) {
    //     // With real sensors, you would also pass the current gyro heading here.
    //     odometry.resetPosition(new Rotation2d(), 0.0, 0.0, newPose);
    // }

    @Override
    public void periodic() {
        // ============================
        // Odometry update (TODO)
        // ============================
        // Mentor note:
        // - Once encoders + gyro are wired, update odometry here:
        //
        // Rotation2d heading = getGyroHeading();
        // double leftMeters = getLeftEncoderMeters();
        // double rightMeters = getRightEncoderMeters();
        // odometry.update(heading, leftMeters, rightMeters);
        //
        // For now, pose remains at default (0,0,0) but code compiles and is ready.
    }
}
