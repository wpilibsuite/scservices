package frc.robot;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.SystemServer;

public class ExpansionHubPidConstants {
  private final DoublePublisher pPublisher;
  private final DoublePublisher iPublisher;
  private final DoublePublisher dPublisher;
  private final DoublePublisher sPublisher;
  private final DoublePublisher vPublisher;
  private final DoublePublisher aPublisher;

  private final BooleanPublisher continuousPublisher;
  private final DoublePublisher continuousMinimumPublisher;
  private final DoublePublisher continuousMaximumPublisher;

  public ExpansionHubPidConstants(int hubNumber, int motorNumber, boolean isVelocityPid) {
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

    String pidType = isVelocityPid ? "velocity" : "position";

    pPublisher = systemServer
        .getDoubleTopic("/rhsp/" + hubNumber + "/motor" + motorNumber + "/pid/" + pidType + "/kp")
        .publish(options);

    iPublisher = systemServer
        .getDoubleTopic("/rhsp/" + hubNumber + "/motor" + motorNumber + "/pid/" + pidType + "/ki")
        .publish(options);

    dPublisher = systemServer
        .getDoubleTopic("/rhsp/" + hubNumber + "/motor" + motorNumber + "/pid/" + pidType + "/kd")
        .publish(options);

    aPublisher = systemServer
        .getDoubleTopic("/rhsp/" + hubNumber + "/motor" + motorNumber + "/pid/" + pidType + "/ka")
        .publish(options);

    vPublisher = systemServer
        .getDoubleTopic("/rhsp/" + hubNumber + "/motor" + motorNumber + "/pid/" + pidType + "/kv")
        .publish(options);

    sPublisher = systemServer
        .getDoubleTopic("/rhsp/" + hubNumber + "/motor" + motorNumber + "/pid/" + pidType + "/ks")
        .publish(options);

    continuousPublisher = systemServer
        .getBooleanTopic("/rhsp/" + hubNumber + "/motor" + motorNumber + "/pid/" + pidType + "/continuous")
        .publish(options);

    continuousMinimumPublisher = systemServer
        .getDoubleTopic("/rhsp/" + hubNumber + "/motor" + motorNumber + "/pid/" + pidType + "/continuousMinimum")
        .publish(options);

    continuousMaximumPublisher = systemServer
        .getDoubleTopic("/rhsp/" + hubNumber + "/motor" + motorNumber + "/pid/" + pidType + "/continousMaximum")
        .publish(options);
  }

  public void setPID(double p, double i, double d) {
    pPublisher.set(p);
    iPublisher.set(i);
    dPublisher.set(d);
  }

  public void setFF(double s, double v, double a) {
    sPublisher.set(s);
    vPublisher.set(v);
    aPublisher.set(a);
  }

  public void enableContinuousInput(double minimum, double maximum) {
    continuousMaximumPublisher.set(maximum);
    continuousMinimumPublisher.set(minimum);
    continuousPublisher.set(true);
  }

  public void disableContinuousInput() {
    continuousPublisher.set(false);
  }
}
