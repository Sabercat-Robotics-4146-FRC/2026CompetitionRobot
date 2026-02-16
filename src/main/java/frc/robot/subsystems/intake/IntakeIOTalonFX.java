package frc.robot.subsystems.intake;
import static frc.robot.Constants.RobotDevices.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PowerConstants;
import frc.robot.subsystems.flywheel_example.FlywheelIO.FlywheelIOInputs;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.RBSIEnum.CTREPro;

public class IntakeIOTalonFX implements IntakeIO{

  private final TalonFX motor = 
  new TalonFX(Intake.getDeviceNumber(), Intake.getCANBus());

  public final int[] powerPorts = {
    Intake.getPowerPort()
  };

   private final StatusSignal<Angle> position = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = motor.getMotorVoltage();
  private final StatusSignal<Current> current = motor.getSupplyCurrent();
  
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final boolean isCTREPro = Constants.getPhoenixPro() == CTREPro.LICENSED;

  public IntakeIOTalonFX(){
    config.CurrentLimits.SupplyCurrentLimit = PowerConstants.kMotorPortMaxCurrent; 
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode =
        switch (IntakeConstants.kIntakeIdleMode) {
          case COAST -> NeutralModeValue.Coast;
          case BRAKE -> NeutralModeValue.Brake;
        };
    OpenLoopRampsConfigs openRamps = new OpenLoopRampsConfigs();
    openRamps.DutyCycleOpenLoopRampPeriod = IntakeConstants.kIntakeOpenLoopRampPeriod;
    openRamps.VoltageOpenLoopRampPeriod = IntakeConstants.kIntakeOpenLoopRampPeriod;
    openRamps.TorqueOpenLoopRampPeriod = IntakeConstants.kIntakeOpenLoopRampPeriod;
    ClosedLoopRampsConfigs closedRamps = new ClosedLoopRampsConfigs();
    closedRamps.DutyCycleClosedLoopRampPeriod = IntakeConstants.kIntakeClosedLoopRampPeriod;
    closedRamps.VoltageClosedLoopRampPeriod = IntakeConstants.kIntakeClosedLoopRampPeriod;
    closedRamps.TorqueClosedLoopRampPeriod = IntakeConstants.kIntakeClosedLoopRampPeriod;

    // Apply the open- and closed-loop ramp configuration for current smoothing
    config.withClosedLoopRamps(closedRamps).withOpenLoopRamps(openRamps);
    // set Motion Magic Velocity settings
    var motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration =
        400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

    // Apply the configurations to the flywheel motors
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
    // If follower rotates in the opposite direction, set "MotorAlignmentValue" to Opposed

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, current);
    motor.optimizeBusUtilization();
  }


  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        position, velocity, appliedVolts, current);
    inputs.positionRad =
        Units.rotationsToRadians(position.getValueAsDouble()) / IntakeConstants.kIntakeGearRatio;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(velocity.getValueAsDouble()) / IntakeConstants.kIntakeGearRatio;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {current.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double volts) {
    final MotionMagicVoltage m_request = new MotionMagicVoltage(volts);
    m_request.withEnableFOC(isCTREPro);
    motor.setControl(m_request);
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    // create a Motion Magic Velocity request, voltage output
    final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
    m_request.withEnableFOC(isCTREPro);
    motor.setControl(m_request.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
  }

  @Override
  public void setPercent(double percent) {
    // create a Motion Magic DutyCycle request, voltage output
    final MotionMagicDutyCycle m_request = new MotionMagicDutyCycle(percent);
    m_request.withEnableFOC(isCTREPro);
    motor.setControl(m_request);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  /**
   * Set the gains of the Slot0 closed-loop configuration
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Differential gain
   * @param kS Static gain
   * @param kV Velocity gain
   */
  @Override
  public void configureGains(double kP, double kI, double kD, double kS, double kV) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = 0.0;
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
  }

  /**
   * Set the gains of the Slot0 closed-loop configuration
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Differential gain
   * @param kS Static gain
   * @param kV Velocity gain
   * @param kA Acceleration gain
   */
  @Override
  public void configureGains(double kP, double kI, double kD, double kS, double kV, double kA) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
  }
}

