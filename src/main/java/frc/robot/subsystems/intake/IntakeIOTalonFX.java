package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.RobotDevices.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.util.RBSIEnum.CTREPro;


public class IntakeIOTalonFX implements IntakeIO {

  public enum Speed {
    FEED(5000);

    private final double rpm;

    private Speed(double rpm) {
      this.rpm = rpm;
    }

    public AngularVelocity angularVelocity() {
      return RPM.of(rpm);
    }
  }

  private final TalonFX roller;
  private final SparkMax extender;
  private final VoltageOut voltageRequest = new VoltageOut(6);
  public final int[] powerPorts = {IntakeRoller.getPowerPort(), IntakeExtender.getPowerPort()};

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final boolean isCTREPro = (Constants.getPhoenixPro() == CTREPro.LICENSED);
  private final SparkMaxConfig maxConfig = new SparkMaxConfig();


  public IntakeIOTalonFX() {
    roller = new TalonFX(IntakeRoller.getDeviceNumber(), IntakeRoller.getCANBus());
    extender = new SparkMax(IntakeExtender.getDeviceNumber(), MotorType.kBrushed);

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    roller.getConfigurator().apply(config);
 
    maxConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
    maxConfig.smartCurrentLimit(50);

    extender.configure(
            maxConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
  }

  @Override
  public void setOutputExtender(double n) {
    extender.set(n);
  }

  @Override
  public void setOutputRoller() {
    roller.setControl(voltageRequest);
  }

  @Override
  public void stopExtender() {
    extender.set(0);
  }

  @Override
  public void stopRoller() {
    roller.stopMotor();
  }

  @Override
  public void setMode() {
    roller.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setExtenderMode(boolean enabled){
    maxConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    extender.configure(
            maxConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
  }
}
