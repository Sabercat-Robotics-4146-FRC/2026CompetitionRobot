package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO extends RBSIO{

  @AutoLog
  public static class IntakeIOInputs(){
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(IntakeIO io){} 

  public default void setVelocity(double velocityRadPerSec){}

  public default void configureGains(double kP, double kI, double kD, double kS, double kV){}



}
