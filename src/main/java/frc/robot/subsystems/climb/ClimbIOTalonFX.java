
package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.RobotDevices;

public class ClimbIOTalonFX implements ClimbIO {

  private TalonFX climbMotor;
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0);

  private StatusSignal<Angle> position;
  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<Voltage> appliedVolts;
  private StatusSignal<Current> current;

  private final double kSpringFF = 10;

  public ClimbIOTalonFX() {
    climbMotor =
        new TalonFX(
            RobotDevices.CLIMB_MOTOR.getDeviceNumber(), RobotDevices.CLIMB_MOTOR.getCANBus());

    position = climbMotor.getPosition();
    velocity = climbMotor.getVelocity();
    appliedVolts = climbMotor.getMotorVoltage();
    current = climbMotor.getSupplyCurrent();

    configureClimbMotor();
  }

  public void configureClimbMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // PID gains
    config.Slot0.kP = 0.5; // tune
    config.Slot0.kI = 0;
    config.Slot0.kD = 0.2;

    // Feedforward
    config.Slot0.kS = 0.1;
    config.Slot0.kV = 0.1;
    config.Slot0.kG = 0;

    // Current limits
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    climbMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);
    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble()) / 80;
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble()) / 80;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {current.getValueAsDouble()};
  }

  @Override
  public double getPosition() {
    return Units.rotationsToRadians(position.getValueAsDouble()) / 80;
  }

  @Override
  public void zeroPosition() {
    climbMotor.setPosition(0.0);
  }

  @Override
  public void setMode(boolean mode) {
    climbMotor.setNeutralMode(mode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setPosition() {
    climbMotor.setControl(motionMagicVoltage.withPosition(20));
  }

  @Override
  public void goHome() {
    setMode(false);
    climbMotor.setControl(voltageRequest.withOutput(2));
  }

  @Override
  public void goUp() {
    setMode(false); // release brake
    climbMotor.setControl(motionMagicVoltage.withPosition(-15).withFeedForward(2));
  }

  @Override
  public void positive() {
    climbMotor.setControl(new VoltageOut(4));
  }

  @Override
  public void negative() {
    climbMotor.setControl(new VoltageOut(-4));
  }

  @Override
  public void setVoltage(double volts) {
    climbMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void stop() {
    climbMotor.stopMotor();
    setMode(true);
  }
}
