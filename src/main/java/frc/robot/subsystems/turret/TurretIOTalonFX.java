// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.turret;

import static frc.robot.Constants.RobotDevices.*;
import static frc.robot.Constants.TurretConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.PowerConstants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.RBSIEnum.CTREPro;

public class TurretIOTalonFX implements TurretIO {

  // Define the leader / follower motors from the Ports section of RobotContainer
  private final TalonFX motor = new TalonFX(TURRET.getDeviceNumber(), TURRET.getCANBus());
  // IMPORTANT: Include here all devices listed above that are part of this mechanism!
  public final int[] powerPorts = {TURRET.getPowerPort()};

  private final StatusSignal<Angle> motorPosition = motor.getPosition();
  private final StatusSignal<AngularVelocity> motorVelocity = motor.getVelocity();
  private final StatusSignal<Voltage> motorAppliedVolts = motor.getMotorVoltage();
  private final StatusSignal<Current> motorCurrent = motor.getSupplyCurrent();

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final boolean isCTREPro = Constants.getPhoenixPro() == CTREPro.LICENSED;

  public TurretIOTalonFX() {
    config.CurrentLimits.SupplyCurrentLimit = PowerConstants.kMotorPortMaxCurrent;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.Feedback.SensorToMechanismRatio = kTurretGearRatio;
    config.MotorOutput.NeutralMode =
        switch (kTurretIdleMode) {
          case COAST -> NeutralModeValue.Coast;
          case BRAKE -> NeutralModeValue.Brake;
        };

    // Build the OpenLoopRampsConfigs and ClosedLoopRampsConfigs for current smoothing
    OpenLoopRampsConfigs openRamps = new OpenLoopRampsConfigs();
    openRamps.DutyCycleOpenLoopRampPeriod = kTurretOpenLoopRampPeriod;
    openRamps.VoltageOpenLoopRampPeriod = kTurretOpenLoopRampPeriod;
    openRamps.TorqueOpenLoopRampPeriod = kTurretOpenLoopRampPeriod;
    ClosedLoopRampsConfigs closedRamps = new ClosedLoopRampsConfigs();
    closedRamps.DutyCycleClosedLoopRampPeriod = kTurretClosedLoopRampPeriod;
    closedRamps.VoltageClosedLoopRampPeriod = kTurretClosedLoopRampPeriod;
    closedRamps.TorqueClosedLoopRampPeriod = kTurretClosedLoopRampPeriod;
    // Apply the open- and closed-loop ramp configuration for current smoothing
    config.withClosedLoopRamps(closedRamps).withOpenLoopRamps(openRamps);
    // set Motion Magic Velocity settings
    var motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = 1; // These are in mechanism units
    motionMagicConfigs.MotionMagicCruiseVelocity = 1;

    // Apply the configurations to the flywheel motors
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);
    inputs.positionRad = Units.rotationsToRadians(motor.getPosition().getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(motorVelocity.getValueAsDouble());
    inputs.appliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {motorCurrent.getValueAsDouble()};
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
  public void runPosition(double mechanismPositionRad) {
    // create a Motion Magic Voltage request, voltage output
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    m_request.withEnableFOC(isCTREPro);
    motor.setControl(m_request.withPosition(Units.radiansToRotations(mechanismPositionRad)));
  }

  @Override
  public void setPosition(double positionRad) {
    motor.setPosition(Units.radiansToRotations(positionRad));
  }

  @Override
  public void zeroPosition() {
    motor.setPosition(0.0);
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
