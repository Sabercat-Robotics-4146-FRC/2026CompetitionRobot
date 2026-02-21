package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class ShooterIOTalonFX {

  private final TalonFX shooter;

  public ShooterIOTalonFX(){
    shooter = new TalonFX(Constants.RobotDevices.Shooter.getDeviceNumber(), Constants.RobotDevices.Shooter.getBus());
  }


  



}
