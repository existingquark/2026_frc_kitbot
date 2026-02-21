/* 
 * QuestNavIONT4: Interface for the QuestNavSubsystem using NT4.
 * 
 * [Put summary and notes here]
 *  
 * 
 * 
 */

package frc.robot.subsystems.QuestNavSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;

public final class QuestNavIONT4 implements QuestNavIO {
  private final DoubleSubscriber xSub;
  private final DoubleSubscriber ySub;
  private final DoubleSubscriber yawSub;
  private final DoubleSubscriber tsSub;
  private final DoubleSubscriber qualitySub;
  private final BooleanSubscriber connectedSub;
  private final StringSubscriber frameSub;

  private Optional<QuestNavPose> latest = Optional.empty();

  // rioTime ~= jetsonTs + offset
  private boolean offsetInitialized = false;
  private double jetsonToRioOffset = 0.0;

  public QuestNavIONT4() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable(QuestNavConstants.TABLE);

    xSub = table.getDoubleTopic(QuestNavConstants.KEY_X_M).subscribe(Double.NaN);
    ySub = table.getDoubleTopic(QuestNavConstants.KEY_Y_M).subscribe(Double.NaN);
    yawSub = table.getDoubleTopic(QuestNavConstants.KEY_YAW_RAD).subscribe(Double.NaN);
    tsSub = table.getDoubleTopic(QuestNavConstants.KEY_TS_S).subscribe(Double.NaN);
    qualitySub = table.getDoubleTopic(QuestNavConstants.KEY_QUALITY).subscribe(0.0);
    connectedSub = table.getBooleanTopic(QuestNavConstants.KEY_CONNECTED).subscribe(false);
    frameSub = table.getStringTopic(QuestNavConstants.KEY_FRAME).subscribe("");
  }

  @Override
  public void periodic() {
    double x = xSub.get();
    double y = ySub.get();
    double yawRad = yawSub.get();
    double jetsonTs = tsSub.get();
    double quality = qualitySub.get();
    boolean connected = connectedSub.get();
    String frame = frameSub.get();

    if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(yawRad) || Double.isNaN(jetsonTs)) {
      return;
    }

    double rioNow = Timer.getFPGATimestamp();

    if (!offsetInitialized) {
      jetsonToRioOffset = rioNow - jetsonTs;
      offsetInitialized = true;
    } else {
      double newOffset = rioNow - jetsonTs;
      jetsonToRioOffset = 0.95 * jetsonToRioOffset + 0.05 * newOffset;
    }

    latest =
        Optional.of(
            new QuestNavPose(
                x, y, new Rotation2d(yawRad), jetsonTs, quality, connected, frame));
  }

  @Override
  public Optional<QuestNavPose> getLatest() {
    return latest;
  }

  @Override
  public Optional<Double> getLatestRioTimestampSeconds() {
    if (latest.isEmpty() || !offsetInitialized) return Optional.empty();
    return Optional.of(latest.get().timestampSeconds + jetsonToRioOffset);
  }

  @Override
  public QuestNavHealth getHealth() {
    if (latest.isEmpty() || !offsetInitialized) {
      return new QuestNavHealth(false, Double.POSITIVE_INFINITY, true, 0.0);
    }

    double rioNow = Timer.getFPGATimestamp();
    double measRioTs = latest.get().timestampSeconds + jetsonToRioOffset;
    double age = rioNow - measRioTs;

    boolean stale = age > QuestNavConstants.STALE_TIMEOUT_S;
    return new QuestNavHealth(latest.get().connected, age, stale, latest.get().quality);
  }
}

