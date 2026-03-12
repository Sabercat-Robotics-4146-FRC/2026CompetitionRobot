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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.util.RBSISubsystem;
import java.util.function.Supplier;
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
  private Pose3d hubTarget;

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io, Supplier<Pose2d> posesSupplier) {
    this.io = io;
    this.pose = posesSupplier;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getMode()) {
      case REAL:
        io.configureGains(12.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        break;
      case REPLAY:
        io.configureGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        break;
      case SIM:
      default:
        io.configureGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        break;
    }

    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      hubTarget = kHubTargetRed;
    } else {
      hubTarget = kHubTargetBlue;
    }
  }

  /** Periodic function -- inherits timing logic from RBSISubsystem */
  @Override
  protected void rbsiPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/State", shooterState);
    Logger.recordOutput("Shooter/IsAtSetpoint", isAtSetpoint());
    Logger.recordOutput("Shooter/RPS", getVelocityRPS());

    if (shooterState == ShooterState.IDLE) {
      stop();
    }
    if (shooterState == ShooterState.SHOOTING) {
      runVelocity(calculateTargetRPS());
      Logger.recordOutput("Shooter/CalculatedRPS", calculateTargetRPS());
    }
    if (shooterState == ShooterState.OVERRIDE) {
      runVelocity(maxVelocity);
    }

    Logger.recordOutput("Shooter/DistanceToHub", getDistanceToTarget());
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPS) {
    io.setVelocity(velocityRPS);

    // Log Shooter setpoint
    Logger.recordOutput("Shooter/SetpointRPS", velocityRPS);
  }

  /** Stops the shooter. */
  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in RPS. */
  @AutoLogOutput(key = "Mechanism/Shooter")
  public double getVelocityRPS() {
    return 0.0166667 * Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  public void setShooterState(ShooterState state) {
    shooterState = state;
  }

  private static final Transform2d ROBOT_TO_SHOOTER =
      new Transform2d(Units.inchesToMeters(-5.25), 0.0, new Rotation2d());

  public double getDistanceToTarget() {
    Pose2d shooterPose = pose.get().plus(ROBOT_TO_SHOOTER);
    return shooterPose.getTranslation().getDistance(hubTarget.toPose2d().getTranslation());
  }

  private double calculateTargetRPS() {
    return (-7.67 * getDistanceToTarget() + -52.8);
  }

  public boolean isAtSetpoint() {
    Logger.recordOutput("Shooter/SetpointDiff", Math.abs(getVelocityRPS() - calculateTargetRPS()));
    return (Math.abs(getVelocityRPS() - calculateTargetRPS()) < 2);
  }

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
