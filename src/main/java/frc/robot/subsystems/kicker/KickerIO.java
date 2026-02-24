package frc.robot.subsystems.kicker;

public interface KickerIO {

  public default void setVoltage(double voltage) {}

  public default void stop() {}
}
