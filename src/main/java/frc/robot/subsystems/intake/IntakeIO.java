package frc.robot.subsystems.intake;

import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO extends RBSIIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double extenderpositionRad = 0.0;
    public double extendervelocityRadPerSec = 0.0;
    public double extenderappliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputsRoller(IntakeIOInputs io) {}

  public default void updateInputsExtender(IntakeIOInputs io) {}

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
