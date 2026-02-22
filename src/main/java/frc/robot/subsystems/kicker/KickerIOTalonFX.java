package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class KickerIOTalonFX {
  private final TalonFX kicker; 

  public KickerIOTalonFX(){
    kicker = new TalonFX(Constants.RobotDevices.Kicker.getDeviceNumber(), Constants.RobotDevices.Kicker.getBus());

  }

}
