/* 
 * QuestNavHealth: Represents the health status of the QuestNavSubsystem.
 * 
 * [Put summary and notes here]
 *  
 * 
 * 
 */

package frc.robot.subsystems.QuestNavSubsystem;

public final class QuestNavHealth {
  public final boolean connected;
  public final double ageSeconds;
  public final boolean stale;
  public final double quality;

  public QuestNavHealth(boolean connected, double ageSeconds, boolean stale, double quality) {
    this.connected = connected;
    this.ageSeconds = ageSeconds;
    this.stale = stale;
    this.quality = quality;
  }
}
