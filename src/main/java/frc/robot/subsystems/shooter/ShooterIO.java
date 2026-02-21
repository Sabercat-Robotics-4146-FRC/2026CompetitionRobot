package frc.robot.subsystems.shooter;

public interface ShooterIO {

  public default void velocityRPM(double velocityRPM){}

  public default void configurePID(double kP, double kI, double kD){}

  public default void setMode(boolean mode){}

  public default void percentOutput(double percentOutput){}

  public default void stop(){}

  public default void getCurrent(){}
  



}
