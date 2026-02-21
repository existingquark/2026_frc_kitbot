/* 
 * QuestNavIO: Interface for the QuestNavSubsystem.
 * 
 * [Put summary and notes here]
 *  
 * 
 * 
 */

package frc.robot.subsystems.QuestNavSubsystem;

import java.util.Optional;

public interface QuestNavIO {
  Optional<QuestNavPose> getLatest();
  QuestNavHealth getHealth();
  Optional<Double> getLatestRioTimestampSeconds();
  void periodic();
}
