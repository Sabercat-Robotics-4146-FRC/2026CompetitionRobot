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
import frc.robot.util.LoggedTunableNumber;

public class ClimbIOTalonFX implements ClimbIO {

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Climb/kP", 0.55);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Climb/kI", 0.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Climb/kD", 0.0);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Climb/kS", 0.55);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Climb/kV", 0.55);

  public double homedPosition = 0;
  public static double hangedPosition = 100; // needs tuning these are encoder values
  public double voltage;
  private TalonFX climbMotor;
  private final VoltageOut voltageRequest = new VoltageOut(voltage);
  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0);

  private StatusSignal<Angle> position;
  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<Voltage> appliedVolts;
  private StatusSignal<Current> current;

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
  public void setMode(boolean mode) {
    climbMotor.setNeutralMode(mode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void goHome() {
    climbMotor.setControl(new VoltageOut(9));
    setMode(true);
  }

  public void configureClimbMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // pid gains
    config.Slot0.kP = 0.5; // tune this
    config.Slot0.kI = 0;
    config.Slot0.kD = 0.2;

    // Feedforward (helps shooter reach speed faster)
    config.Slot0.kS = 0.1; // static friction
    config.Slot0.kV = 0.1; // velocity gain

    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // config.MotionMagic.MotionMagicCruiseVelocity = 200; // tune it
    // config.MotionMagic.MotionMagicAcceleration = 100; // tune it

    climbMotor.getConfigurator().apply(config);
  }

  @Override
  public void goUp() {

    climbMotor.setControl(new VoltageOut(-2));
    /*
    motionMagicVoltage
        .withPosition(Units.radiansToRotations(hangedPosition))
        .withFeedForward(0));*/
  }

  public void zeroPosition() {
    climbMotor.setPosition(0);
  }

  @Override
  public void stop() {
    climbMotor.stopMotor();
  }

  public void zeroPosition() {
    climbMotor.setPosition(0);
  }

  @Override
  public void stop() {
    climbMotor.stopMotor();
  }
}
