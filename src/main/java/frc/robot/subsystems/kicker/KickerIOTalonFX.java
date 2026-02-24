package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class KickerIOTalonFX implements KickerIO {
  private final TalonFX kicker;
  private final VoltageOut voltageOut = new VoltageOut(0);

  private final TalonFXConfiguration config =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(120))
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(50))
                  .withSupplyCurrentLimitEnable(true));

  public KickerIOTalonFX() {
    kicker =
        new TalonFX(
            Constants.RobotDevices.Kicker.getDeviceNumber(),
            Constants.RobotDevices.Kicker.getBus());
    kicker.getConfigurator().apply(config);
  }

  @Override
  public void setVoltage(double voltage) {
    kicker.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void stop() {
    kicker.setControl(voltageOut.withOutput(0));
  }
}
