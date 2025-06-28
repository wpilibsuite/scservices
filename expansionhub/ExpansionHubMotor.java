package frc.robot;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.SystemServer;

public class ExpansionHubMotor {
  private final DoubleSubscriber encoderSubscriber;
  private final DoubleSubscriber encoderVelocitySubscriber;
  private final DoubleSubscriber currentSubscriber;

  private final DoublePublisher setpointPublisher;
  private final BooleanPublisher floatOn0Publisher;
  private final BooleanPublisher enabledPublisher;

  private final IntegerPublisher modePublisher;

  private final BooleanPublisher reversedPublisher;
  private final BooleanPublisher resetEncoderPublisher;

  private final DoublePublisher distancePerCountPublisher;

  private final BooleanSubscriber hubConnectedSubscriber;

  public ExpansionHubMotor(int hubNumber, int motorNumber) {
    if (hubNumber < 0 || hubNumber > 3) {
      throw new IllegalArgumentException(
          "Hub number out of range, must be between 0 and 3 inclusive (Matching USB numbers)");
    }

    if (motorNumber < 0 || motorNumber > 3) {
      throw new IllegalArgumentException(
          "Motor number out of range, must be between 0 and 3 inclusive");
    }

    NetworkTableInstance systemServer = SystemServer.getSystemServer();

    PubSubOption[] options = new PubSubOption[] { PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true),
        PubSubOption.periodic(0.005) };

    encoderSubscriber = systemServer
        .getDoubleTopic("/rhsp/" + hubNumber + "/motor" + motorNumber + "/encoder")
        .subscribe(0, options);
    encoderVelocitySubscriber = systemServer
        .getDoubleTopic("/rhsp/" + hubNumber + "/motor" +
            motorNumber + "/encoderVelocity")
        .subscribe(0, options);
    currentSubscriber = systemServer
        .getDoubleTopic("/rhsp/" + hubNumber + "/motor" + motorNumber + "/current")
        .subscribe(0, options);

    hubConnectedSubscriber = systemServer.getBooleanTopic("/rhsp/" + hubNumber + "/connected").subscribe(false);

    setpointPublisher = systemServer
        .getDoubleTopic("/rhsp/" + hubNumber + "/motor" + motorNumber + "/setpoint")
        .publish(options);

    distancePerCountPublisher = systemServer
        .getDoubleTopic("/rhsp/" + hubNumber + "/motor" + motorNumber + "/distancePerCount")
        .publish(options);

    floatOn0Publisher = systemServer
        .getBooleanTopic("/rhsp/" + hubNumber + "/motor" +
            motorNumber + "/floatOn0")
        .publish(options);
    enabledPublisher = systemServer
        .getBooleanTopic("/rhsp/" + hubNumber + "/motor" + motorNumber + "/enabled")
        .publish(options);

    modePublisher = systemServer
        .getIntegerTopic("/rhsp/" + hubNumber + "/motor" + motorNumber + "/mode")
        .publish(options);

    reversedPublisher = systemServer
        .getBooleanTopic("/rhsp/" + hubNumber + "/motor" +
            motorNumber + "/reversed")
        .publish(options);

    resetEncoderPublisher = systemServer
        .getBooleanTopic("/rhsp/" + hubNumber + "/motor" + motorNumber +
            "/resetEncoder")
        .publish(options);
  }

  public void setPercentagePower(double power) {
    modePublisher.set(0);
    setpointPublisher.set(power);
  }

  public void setEnabled(boolean enabled) {
    enabledPublisher.set(enabled);
  }

  public void setFloatOn0(boolean floatOn0) {
    floatOn0Publisher.set(floatOn0);
  }

  public double getCurrent() {
    return currentSubscriber.get(0);
  }

  public void setDistancePerCount(double perCount) {
    distancePerCountPublisher.set(perCount);
  }

  public boolean isHubConnected() {
    return hubConnectedSubscriber.get(false);
  }

  public double getEncoderVelocity() {
    return encoderVelocitySubscriber.get(0);
  }

  public double getEncoder() {
    return encoderSubscriber.get(0);
  }

  public void setReversed(boolean reversed) {
    reversedPublisher.set(reversed);
  }

  public void resetEncoder() {
    resetEncoderPublisher.set(true);
  }
}
