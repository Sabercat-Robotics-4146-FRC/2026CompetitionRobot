package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.RobotDevices.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RBSIEnum.CTREPro;

public class IntakeIOTalonFX implements IntakeIO {

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/ kP", 0.55);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Intake/ kI", 0.55);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/ kD", 0.55);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Intake/ kS", 0.55);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Intake/ kV", 0.55);
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Intake/ kA", 0.55);
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Intake/ kG", 0.55);

  public enum Position {
    HOMED(110),
    INTAKE(-4);

    private final double degrees;

    private Position(double degrees) {
      this.degrees = degrees;
    }

    public Angle angle() {
      return Degrees.of(degrees);
    }
  }

  private final TalonFX roller;
  private final TalonFX extender;
  private final VoltageOut voltageRequest = new VoltageOut(6);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
  public final int[] powerPorts = {IntakeRoller.getPowerPort(), IntakeExtender.getPowerPort()};

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final TalonFXConfiguration secondConfig = new TalonFXConfiguration();
  private final boolean isCTREPro = (Constants.getPhoenixPro() == CTREPro.LICENSED);

  // private static final AngularVelocity kMaxPivotSpeed =
  // KrakenX60.kFreeSpeed.div(kPivotReduction);

  public IntakeIOTalonFX() {
    roller = new TalonFX(IntakeRoller.getDeviceNumber(), IntakeRoller.getCANBus());
    extender = new TalonFX(IntakeExtender.getDeviceNumber(), IntakeExtender.getCANBus());

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    roller.getConfigurator().apply(config);

    // pid gains
    secondConfig.Slot0.kP = kP.get(); // tune this
    secondConfig.Slot0.kI = kI.get();
    secondConfig.Slot0.kD = kD.get();

    // Feedforward
    secondConfig.Slot0.kS = kS.get(); // static friction
    secondConfig.Slot0.kV = kV.get(); // velocity gain
    secondConfig.Slot0.kG = kG.get();

    secondConfig.CurrentLimits.StatorCurrentLimit = 40;
    secondConfig.MotionMagic.MotionMagicCruiseVelocity = 100; // tune it
    secondConfig.MotionMagic.MotionMagicAcceleration = 100; // tune it

    extender.getConfigurator().apply(secondConfig);
  }

  /*   @Override
  public void setOutputExtender(double n) {
    extender.
  }*/

  @Override
  public void setOutputRoller() {
    roller.setControl(voltageRequest);
  }

  public void set(Position position) {
    extender.setControl(motionMagicRequest.withPosition(position.angle()));
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
