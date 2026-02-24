package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class ShooterIOTalonFX implements ShooterIO {

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", 0.0);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/kI", 0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD", 0);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV", 0.0);
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Shooter/kA", 0.0);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/kS", 0.0);

  private final TalonFX shooter;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final double targetRPS = 0;

  public ShooterIOTalonFX() {
    shooter =
        new TalonFX(
            Constants.RobotDevices.Shooter.getDeviceNumber(),
            Constants.RobotDevices.Shooter.getBus());

    configureShooter();
  }

  @Override
  public void setVelocityRPS() {
    shooter.setControl(velocityRequest.withVelocity(targetRPS));
  }

  public void configureShooter() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // PID gains
    config.Slot0.kP = kP.get(); // tune this
    config.Slot0.kI = kI.get();
    config.Slot0.kD = kD.get();

    // Feedforward (helps shooter reach speed faster)
    config.Slot0.kS = kS.get(); // static friction
    config.Slot0.kV = kV.get(); // velocity gain
    config.Slot0.kA = kA.get();

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
