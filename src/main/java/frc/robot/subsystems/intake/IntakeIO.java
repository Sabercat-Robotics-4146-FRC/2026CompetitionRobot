package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.RBSIIO;

public interface IntakeIO extends RBSIIO{

  @AutoLog
  public static class IntakeIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(IntakeIO io){} 

  public default void setVelocity(double velocityRadPerSec){}

  public default void configureGains(double kP, double kI, double kD, double kS, double kV){}

  public default void configureGains(
      double kP, double kI, double kD, double kS, double kV, double kA) {}



}
