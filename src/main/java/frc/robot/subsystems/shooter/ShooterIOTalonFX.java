package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.Amps;

import frc.robot.Constants;

public class ShooterIOTalonFX implements ShooterIO{

  private final TalonFX shooter;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0); 
  private final double targetRPS = 0; 

  public ShooterIOTalonFX(){
    shooter = new TalonFX(Constants.RobotDevices.Shooter.getDeviceNumber(), 
    Constants.RobotDevices.Shooter.getBus());

    configureShooter();
  }

  @Override
  public void setVelocityRPS(){
    shooter.setControl(velocityRequest.withVelocity(targetRPS));
  }

  public void configureShooter(){
        TalonFXConfiguration config = new TalonFXConfiguration();

    // PID gains
    config.Slot0.kP = 0.12;  // tune this
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    // Feedforward (helps shooter reach speed faster)
    config.Slot0.kS = 0.2;   // static friction
    config.Slot0.kV = 0.12;  // velocity gain
    config.Slot0.kA = 0.0;

    // mode 
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // current limits
    config.CurrentLimits.StatorCurrentLimit = 120;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.CurrentLimits.SupplyCurrentLimit = 70;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    shooter.getConfigurator().apply(config);
  }


  public double getVelocityRPM() {
    return shooter.getVelocity().getValueAsDouble() * 60;
  }

  @Override
  public void stop() {
    shooter.stopMotor();
  }


}
