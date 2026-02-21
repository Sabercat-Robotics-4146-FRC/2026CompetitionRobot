package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

  // -- Power Ports used by Climb -- //
  public final int[] powerPorts = {};

  @AutoLog
  public static class ClimbIOInputs {
    public boolean motorConnected = true;
    public double velocityRadPerSec = 0.0;
    public double[] appliedVolts = new double[] {};
    public double[] torqueCurrentAmps = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] tempCelsius = new double[] {};
  }

  default void updateInputs(ClimbIOInputs inputs) {}

  default void runOpenLoop(double output) {}

  default void stop() {}

  default void goUp() {}

  default void setMode(boolean enabled) {}

  default void goHome(double percent) {}

  default double getPosition() {
    return 0.0;
  }

  default void runTone(double tone) {}

  // default double getCurrent() {}
  ;
}
