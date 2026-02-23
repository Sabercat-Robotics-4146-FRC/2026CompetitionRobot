package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants.RobotDevices;
import frc.robot.util.LoggedTunableNumber;

public class ClimbIOTalonFX implements ClimbIO {

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Climb/kP", 0.55);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Climb/kI", 0.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Climb/kD", 0.0);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Climb/kS", 0.55);
   private static final LoggedTunableNumber kV = new LoggedTunableNumber("Climb/kV", 0.55);
  public enum ClimbState{
    HOMED,
    HANGED
  }
  public double homedPosition = 0; 
  public double hangedPosition = 100; //needs tuning 

  private TalonFX climbMotor;
  private final DutyCycleOut percentRequest = new DutyCycleOut(0.0);

  public ClimbIOTalonFX() {
    climbMotor =
        new TalonFX(
            RobotDevices.CLIMB_MOTOR.getDeviceNumber(), RobotDevices.CLIMB_MOTOR.getCANBus());
  }


  @Override
  public void setMode(boolean mode) {
    climbMotor.setNeutralMode(mode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void goHome(double percent) {
    climbMotor.setControl(percentRequest.withOutput(percent));
    setMode(true);
  }

  @Override
  public void goUp() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.kP = kP.get();
    config.kI = kI.get();
    config.kD = kD.get();
    config.kS = kS.get();
    config.kV = kV.get();

    climbMotor.getConfigurator().apply(config);
    climbMotor.setControl(percentRequest.withOutput(0.0));
  }

  public void zeroPosition(){
    climbMotor.setPosition(0);
  }

  public void setBrakeMode(){
  }
}
