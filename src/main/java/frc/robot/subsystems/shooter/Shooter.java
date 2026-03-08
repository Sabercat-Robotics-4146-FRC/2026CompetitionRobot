// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends RBSISubsystem {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public enum ShooterState {
    IDLE,
    SHOOTING,
    OVERRIDE
  }

  public ShooterState shooterState = ShooterState.IDLE;

  private final Supplier<Pose2d> pose;

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io, Supplier<Pose2d> posesSupplier) {
    this.io = io;
    this.pose = posesSupplier;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getMode()) {
      case REAL:
        io.configureGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        break;
      case REPLAY:
        io.configureGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        break;
      case SIM:
      default:
        io.configureGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        break;
    }
  }

  /** Periodic function -- inherits timing logic from RBSISubsystem */
  @Override
  protected void rbsiPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    if (shooterState == ShooterState.IDLE) {
      stop();
    }
    if (shooterState == ShooterState.SHOOTING) {
      runVelocity(calculateTargetRPM());
      Logger.recordOutput("Shooter/CalculatedRPM", calculateTargetRPM());
    }
    if (shooterState == ShooterState.OVERRIDE) {
      runVelocity(maxVelocity);
    }
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec);

    // Log Shooter setpoint
    Logger.recordOutput("Shooter/SetpointRPM", velocityRPM);
  }

  /** Stops the shooter. */
  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput(key = "Mechanism/Shooter")
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  public void setShooterState(ShooterState state) {
    shooterState = state;
  }

  private double calculateTargetRPM(){
    // TODO: Implement a function to calculate the target RPM based on distance to the target and current velocity
    return 0.0;
  }

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
