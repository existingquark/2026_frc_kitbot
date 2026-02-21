/* 
 * QuestNavConstants: Contains constants for the QuestNavSubsystem.
 * 
 * [Put summary and notes here]
 *  
 * 
 * 
 */

 
package frc.robot.subsystems.QuestNavSubsystem;

public final class QuestNavConstants {
  private QuestNavConstants() {}

  public static final String TABLE = "/questnav";

  public static final String KEY_X_M = "x_m";
  public static final String KEY_Y_M = "y_m";
  public static final String KEY_YAW_RAD = "yaw_rad";
  public static final String KEY_TS_S = "ts_s";
  public static final String KEY_QUALITY = "quality";
  public static final String KEY_CONNECTED = "connected";
  public static final String KEY_FRAME = "frame";

  // Gating defaults (tune later)
  public static final double MIN_QUALITY = 0.60;
  public static final double STALE_TIMEOUT_S = 0.20;

  public static final double MAX_POSITION_JUMP_M = 1.00;
  public static final double MAX_YAW_JUMP_RAD = Math.toRadians(90.0);
}
