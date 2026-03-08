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
import frc.robot.util.RBSIEnum.CTREPro;

public class IntakeIOTalonFX implements IntakeIO {

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
  private final VoltageOut voltageRequestOne = new VoltageOut(1.5);
  private final VoltageOut voltageRequestTwo = new VoltageOut(-1.5);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(2);
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
  public void setOutputRoller() {
    roller.setControl(voltageRequest);
    System.out.println("third");
  }

  public void setRetraction() {
    extender.setControl(voltageRequestOne);
  }

  public void setExtender() {
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
