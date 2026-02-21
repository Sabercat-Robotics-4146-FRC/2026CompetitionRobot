package frc.robot.subsystems.climb;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants.RobotDevices;

public class ClimbIOTalonFX implements ClimbIO {

  private TalonFX climbMotor;
  private final Debouncer debouncer = new Debouncer(0.5);
  private final DutyCycleOut percentRequest = new DutyCycleOut(0.0);

  public ClimbIOTalonFX() {
    climbMotor =
        new TalonFX(
            RobotDevices.CLIMB_MOTOR.getDeviceNumber(), RobotDevices.CLIMB_MOTOR.getCANBus());
  }

  /*private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final StatusSignal<AngularVelocity> velocity;

  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;*/

  /*private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

   var motionMagicConfigs = config.MotionMagic;
   motionMagicConfigs.MotionMagicCruiseVelocity = 80;
   motionMagicConfigs.MotionMagicAccleration = 80;

   position = climbMotor.getPosition();
   velocity = climbMotor.getVelocity();
   appliedVolts = climbMotor.getAppliedVoltage();
   torqueCurrent = climbMotor.getTorqueCurrent();

   temp = climbMotor.getTemperature();

   BaseStatusSignal.setUpdateFrequencyForAll(
         50.0, position, velocity, appliedVolts, torqueCurrent, temp);

   motionMagicVoltage.EnableFOC = true;

  */
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
    climbMotor.setControl(percentRequest.withOutput(0.0));
  }
}
