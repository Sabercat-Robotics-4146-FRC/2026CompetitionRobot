package frc.robot.subsystems.climb;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.RobotDevices;

public class ClimbIOTalonFX {

  private TalonFX climbMotor; 

  /* 
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;

  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;*/

  public ClimbIOTalonFX(){
    climbMotor = new TalonFX
    (RobotDevices.CLIMB_MOTOR.getDeviceNumber(), 
    RobotDevices.CLIMB_MOTOR.getCANBus());
  }


  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

  

}
