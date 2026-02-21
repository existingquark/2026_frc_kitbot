

/* 
 * QuestNavPose: Represents the pose of the QuestNavSubsystem.
 * 
 * [Put summary and notes here]
 *  
 * 
 * 
 */

package frc.robot.subsystems.QuestNavSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class QuestNavPose {
  public final double xMeters;
  public final double yMeters;
  public final Rotation2d yaw;
  public final double timestampSeconds; // Jetson monotonic seconds
  public final double quality;          // 0..1
  public final boolean connected;
  public final String frame;

  public QuestNavPose(
      double xMeters,
      double yMeters,
      Rotation2d yaw,
      double timestampSeconds,
      double quality,
      boolean connected,
      String frame) {
    this.xMeters = xMeters;
    this.yMeters = yMeters;
    this.yaw = yaw;
    this.timestampSeconds = timestampSeconds;
    this.quality = quality;
    this.connected = connected;
    this.frame = frame;
  }

  public Pose2d toPose2d() {
    return new Pose2d(xMeters, yMeters, yaw);
  }
}

