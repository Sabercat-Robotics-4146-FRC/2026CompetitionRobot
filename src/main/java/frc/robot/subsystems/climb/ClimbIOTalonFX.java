package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
  public void goHome(double volts) {
    voltage = -6;
    climbMotor.setControl(voltageRequest);
    setMode(true);
  }

  public void configureClimbMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // pid gains
    config.Slot0.kP = kP.get(); // tune this
    config.Slot0.kI = kI.get();
    config.Slot0.kD = kD.get();

    // Feedforward (helps shooter reach speed faster)
    config.Slot0.kS = kS.get(); // static friction
    config.Slot0.kV = kV.get(); // velocity gain

    config.MotionMagic.MotionMagicCruiseVelocity = 200; // tune it
    config.MotionMagic.MotionMagicAcceleration = 100; // tune it

    climbMotor.getConfigurator().apply(config);
  }

  @Override
  public void goUp() {
    voltage = 6;
    configureClimbMotor();
    climbMotor.setControl(voltageRequest);
    /*
    motionMagicVoltage
        .withPosition(Units.radiansToRotations(hangedPosition))
        .withFeedForward(0));*/
  }

  public double getPosition() {
    return climbMotor.getPosition().getValueAsDouble();
  }

  public void zeroPosition() {
    climbMotor.setPosition(0);
  }

  @Override
  public void stop() {
    climbMotor.stopMotor();
  }
}
