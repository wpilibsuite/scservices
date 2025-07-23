package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.SystemServer;

public class ExpansionHubServo {
  private final IntegerPublisher pulseWidthPublisher;
  private final IntegerPublisher framePeriodPublisher;
  private final BooleanPublisher enabledPublisher;

  private final BooleanSubscriber hubConnectedSubscriber;

  private static final double kMaxServoAngle = 180.0;
  private static final double kMinServoAngle = 0.0;

  private static final int kDefaultMaxServoPWM = 2400;
  private static final int kDefaultMinServoPWM = 600;

  private static final int m_minPwm = kDefaultMinServoPWM;
  private static final int m_maxPwm = kDefaultMaxServoPWM;

  public ExpansionHubServo(int hubNumber, int servoNumber) {
    if (hubNumber < 0 || hubNumber > 3) {
      throw new IllegalArgumentException(
          "Hub number out of range, must be between 0 and 3 inclusive (Matching USB numbers)");
    }

    if (servoNumber < 0 || servoNumber > 5) {
      throw new IllegalArgumentException(
          "Servo number out of range, must be between 0 and 5 inclusive");
    }

    NetworkTableInstance systemServer = SystemServer.getSystemServer();

    PubSubOption[] options = new PubSubOption[] { PubSubOption.sendAll(true),
        PubSubOption.keepDuplicates(true),
        PubSubOption.periodic(0.005) };

    hubConnectedSubscriber = systemServer.getBooleanTopic("/rhsp/" + hubNumber + "/connected")
        .subscribe(false);

    pulseWidthPublisher = systemServer
        .getIntegerTopic("/rhsp/" + hubNumber + "/servo" + servoNumber + "/pulseWidth")
        .publish(options);

    pulseWidthPublisher.set(1500);

    framePeriodPublisher = systemServer
        .getIntegerTopic("/rhsp/" + hubNumber + "/servo" + servoNumber + "/framePeriod")
        .publish(options);

    framePeriodPublisher.set(20000);

    enabledPublisher = systemServer
        .getBooleanTopic("/rhsp/" + hubNumber + "/servo" + servoNumber + "/enabled")
        .publish(options);
  }

  /**
   * Set the servo position.
   *
   * <p>
   * Servo values range from 0.0 to 1.0 corresponding to the range of full left to
   * full right.
   *
   * @param value Position from 0.0 to 1.0.
   */
  public void set(double value) {
    value = MathUtil.clamp(value, 0.0, 1.0);

    int rawValue = (int) ((value * getFullRangeScaleFactor()) + m_minPwm);

    setPulseWidth(rawValue);
  }

  /**
   * Set the servo angle.
   *
   * <p>
   * The angles are based on the HS-322HD Servo, and have a range of 0 to 180
   * degrees.
   *
   * <p>
   * Servo angles that are out of the supported range of the servo simply
   * "saturate" in that
   * direction In other words, if the servo has a range of (X degrees to Y
   * degrees) than angles of
   * less than X result in an angle of X being set and angles of more than Y
   * degrees result in an
   * angle of Y being set.
   *
   * @param degrees The angle in degrees to set the servo.
   */
  public void setAngle(double degrees) {
    if (degrees < kMinServoAngle) {
      degrees = kMinServoAngle;
    } else if (degrees > kMaxServoAngle) {
      degrees = kMaxServoAngle;
    }

    set((degrees - kMinServoAngle) / getServoAngleRange());
  }

  private double getFullRangeScaleFactor() {
    return m_maxPwm - m_minPwm;
  }

  private double getServoAngleRange() {
    return kMaxServoAngle - kMinServoAngle;
  }

  public void setPulseWidth(int pulseWidth) {
    pulseWidthPublisher.set(pulseWidth);
  }

  public void setEnabled(boolean enabled) {
    enabledPublisher.set(enabled);
  }

  public void setFramePeriod(int framePeriod) {
    framePeriodPublisher.set(framePeriod);
  }

  public boolean isHubConnected() {
    return hubConnectedSubscriber.get(false);
  }
}
