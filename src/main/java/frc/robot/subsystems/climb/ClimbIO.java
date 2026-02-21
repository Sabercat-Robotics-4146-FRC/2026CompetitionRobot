package frc.robot.subsystems.climb;

import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO extends RBSIIO {

  // -- Power Ports used by Climb -- //
  public final int[] powerPorts = {};

  @AutoLog
  public static class ClimbIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  default void updateInputs(ClimbIOInputs inputs) {}

  default void runOpenLoop(double output) {}

  default void stop() {}

  default void goUp() {}

  default void setMode(boolean enabled) {}

  default void goHome() {}

  default double getPosition() {
    return 0.0;
  }
}
