package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.RBSIIO;

public interface IntakeIO extends RBSIIO{

  @AutoLog
  public static class IntakeIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(IntakeIO io){} 

  public default void setPercentOutputExtender(double percentOutput){}

   public default void setPercentOutputRoller(double percentOutput){}

  public default void stopExtender(){}

  public default void stopRoller(){}

  public default void setMode(){}

  public default void getCurrent(){}



}
