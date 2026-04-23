package frc.robot;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.SystemServer;

public class ExpansionHubPositionConstants {
  private final DoublePublisher m_pPublisher;
  private final DoublePublisher m_iPublisher;
  private final DoublePublisher m_dPublisher;

  private final DoublePublisher m_sPublisher;

  private final BooleanPublisher m_continuousPublisher;
  private final DoublePublisher m_continuousMinimumPublisher;
  private final DoublePublisher m_continuousMaximumPublisher;

  ExpansionHubPositionConstants(int hubNumber, int motorNumber) {
    NetworkTableInstance systemServer = SystemServer.getSystemServer();

    PubSubOption[] options =
        new PubSubOption[] {
          PubSubOption.SEND_ALL, PubSubOption.KEEP_DUPLICATES, PubSubOption.periodic(0.005)
        };

    m_pPublisher =
        systemServer
            .getDoubleTopic("/rhsp/" + hubNumber + "/motor" + motorNumber + "/constants/position" + "/kp")
            .publish(options);

    m_iPublisher =
        systemServer
            .getDoubleTopic("/rhsp/" + hubNumber + "/motor" + motorNumber + "/constants/position" + "/ki")
            .publish(options);

    m_dPublisher =
        systemServer
            .getDoubleTopic("/rhsp/" + hubNumber + "/motor" + motorNumber + "/constants/position" + "/kd")
            .publish(options);

    m_sPublisher =
        systemServer
            .getDoubleTopic("/rhsp/" + hubNumber + "/motor" + motorNumber + "/constants/position" + "/ks")
            .publish(options);

    m_continuousPublisher =
        systemServer
            .getBooleanTopic(
                "/rhsp/" + hubNumber + "/motor" + motorNumber + "/constants/position" + "/continuous")
            .publish(options);

    m_continuousMinimumPublisher =
        systemServer
            .getDoubleTopic(
                "/rhsp/"
                    + hubNumber
                    + "/motor"
                    + motorNumber
                    + "/constants/position"
                    + "/continuousMinimum")
            .publish(options);

    m_continuousMaximumPublisher =
        systemServer
            .getDoubleTopic(
                "/rhsp/"
                    + hubNumber
                    + "/motor"
                    + motorNumber
                    + "/constants/position"
                    + "/continuousMaximum")
            .publish(options);
  }

  public ExpansionHubPositionConstants setPID(double p, double i, double d) {
    m_pPublisher.set(p);
    m_iPublisher.set(i);
    m_dPublisher.set(d);
    return this;
  }

  public ExpansionHubPositionConstants setS(double s) {
    m_sPublisher.set(s);
    return this;
  }

  public ExpansionHubPositionConstants enableContinuousInput(
      double minimumInput, double maximumInput) {
    m_continuousMaximumPublisher.set(maximumInput);
    m_continuousMinimumPublisher.set(minimumInput);
    m_continuousPublisher.set(true);
    return this;
  }

  public ExpansionHubPositionConstants disableContinuousInput() {
    m_continuousPublisher.set(false);
    return this;
  }

  void close() {
    m_pPublisher.close();
    m_iPublisher.close();
    m_dPublisher.close();
    m_sPublisher.close();
    m_continuousPublisher.close();
    m_continuousMinimumPublisher.close();
    m_continuousMaximumPublisher.close();
  }
}
