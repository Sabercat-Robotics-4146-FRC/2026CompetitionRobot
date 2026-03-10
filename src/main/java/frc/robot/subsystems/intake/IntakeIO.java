package frc.robot.subsystems.intake;

import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO extends RBSIIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double[] supplyCurrent = new double[] {};
  }

  public default void updateInputs(IntakeIOInputs io) {}

  public default void setOutputExtender(double output) {}

  public default void setOutputRoller() {}

  public default void setExtender() {}

  public default void setRetraction() {}

  public default void stopExtender() {}

  public default void stopRoller() {}

  public default void setMode() {}

  public default void setExtenderMode(boolean enabled) {}

  public default double getPosition() {
    return 0.0;
  }
}
