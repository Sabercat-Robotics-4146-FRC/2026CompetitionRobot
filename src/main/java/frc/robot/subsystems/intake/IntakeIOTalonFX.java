package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.RobotDevices.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.RBSIEnum.CTREPro;

public class IntakeIOTalonFX implements IntakeIO {

  public enum Position {
    HOMED(5.037),
    INTAKE(-24.8);

    private final double degrees;

    private Position(double degrees) {
      this.degrees = degrees;
    }

    public Angle angle() {
      return Degrees.of(degrees);
    }
  }

  private final TalonFX roller =
      new TalonFX(IntakeRoller.getDeviceNumber(), IntakeRoller.getCANBus());
  private final TalonFX extender =
      new TalonFX(IntakeExtender.getDeviceNumber(), IntakeExtender.getCANBus());
  private final VoltageOut voltageRequest = new VoltageOut(12);
  private final VoltageOut voltageRequestOne = new VoltageOut(1.5);
  private final VoltageOut voltageRequestTwo = new VoltageOut(-1.5);
  public final int[] powerPorts = {IntakeRoller.getPowerPort(), IntakeExtender.getPowerPort()};

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final TalonFXConfiguration secondConfig = new TalonFXConfiguration();
  private final boolean isCTREPro = (Constants.getPhoenixPro() == CTREPro.LICENSED);

  private final StatusSignal<Angle> rollerPosition = roller.getPosition();
  private final StatusSignal<AngularVelocity> rollerVelocity = roller.getVelocity();
  private final StatusSignal<Voltage> rollerAppliedVolts = roller.getMotorVoltage();
  private final StatusSignal<Current> rollerCurrent = roller.getSupplyCurrent();

  private final StatusSignal<Angle> extenderPosition = extender.getPosition();
  private final StatusSignal<AngularVelocity> extenderVelocity = extender.getVelocity();
  private final StatusSignal<Voltage> extenderAppliedVolts = extender.getMotorVoltage();
  private final StatusSignal<Current> extenderCurrent = extender.getSupplyCurrent();

  // private static final AngularVelocity kMaxPivotSpeed =
  // KrakenX60.kFreeSpeed.div(kPivotReduction);

  public IntakeIOTalonFX() {

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    roller.getConfigurator().apply(config);

    // pid gains
    secondConfig.Slot0.kP = 5; // tune this
    secondConfig.Slot0.kI = 0.0;
    secondConfig.Slot0.kD = 0.0;

    // Feedforward
    secondConfig.Slot0.kS = 0.1; // static friction
    secondConfig.Slot0.kV = 0.12; // velocity gain
    secondConfig.Slot0.kG = 5;

    secondConfig.CurrentLimits.StatorCurrentLimit = 120;
    secondConfig.MotionMagic.MotionMagicCruiseVelocity = 100; // tune it
    secondConfig.MotionMagic.MotionMagicAcceleration = 50; // tune it

    extender.getConfigurator().apply(secondConfig);
  }

  @Override
  public void updateInputsRoller(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        rollerPosition, rollerVelocity, extenderAppliedVolts, extenderCurrent);
    inputs.positionRad = Units.rotationsToRadians(rollerPosition.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(rollerVelocity.getValueAsDouble());
    inputs.appliedVolts = rollerAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {rollerCurrent.getValueAsDouble()};
  }

  @Override
  public void updateInputsExtender(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        extenderPosition, extenderVelocity, extenderAppliedVolts, extenderCurrent);
    inputs.extenderpositionRad = Units.rotationsToRadians(extenderPosition.getValueAsDouble());
    inputs.extendervelocityRadPerSec =
        Units.rotationsToRadians(extenderVelocity.getValueAsDouble());
    inputs.extenderappliedVolts = extenderAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {extenderCurrent.getValueAsDouble()};
  }

  @Override
  public void setOutputRoller() {
    roller.setControl(voltageRequest);
    System.out.println("third");
  }

  @Override
  public void setRetraction() {
    // motionMagicRequest = new MotionMagicVoltage(5.037);
    // extender.setControl(motionMagicRequest);
    extender.setControl(voltageRequestOne);
  }

  @Override
  public void setExtender() {
    // motionMagicRequest = new MotionMagicVoltage(-29.8);
    // extender.setControl(motionMagicRequest);
    extender.setControl(voltageRequestTwo);
  }

  @Override
  public double getPosition() {
    return extender.getPosition().getValueAsDouble();
  }

  @Override
  public void stopExtender() {
    extender.stopMotor();
  }

  @Override
  public void stopRoller() {
    roller.stopMotor();
  }

  @Override
  public void setMode() {
    roller.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setExtenderMode(boolean enabled) {
    extender.setNeutralMode(enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake);
  }
}
