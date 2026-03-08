package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.RobotDevices.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
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

  private final TalonFX roller;
  private final TalonFX extender;
  private final VoltageOut voltageRequest = new VoltageOut(9);
  private final VoltageOut voltageRequestOne = new VoltageOut(1.5);
  private final VoltageOut voltageRequestTwo = new VoltageOut(-1.5);
  private MotionMagicVoltage motionMagicRequest;
  public final int[] powerPorts = {IntakeRoller.getPowerPort(), IntakeExtender.getPowerPort()};

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final TalonFXConfiguration secondConfig = new TalonFXConfiguration();
  private final boolean isCTREPro = (Constants.getPhoenixPro() == CTREPro.LICENSED);

  private final StatusSignal<Current> supplyCurrentAmps;

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

    supplyCurrentAmps = extender.getSupplyCurrent();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    boolean connected = BaseStatusSignal.refreshAll(supplyCurrentAmps).isOK();
    inputs.supplyCurrent =
        new double[] {supplyCurrentAmps.getValueAsDouble(), supplyCurrentAmps.getValueAsDouble()};
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
