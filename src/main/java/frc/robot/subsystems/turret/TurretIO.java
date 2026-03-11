// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.turret;

import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO extends RBSIIO {

  @AutoLog
  public static class TurretIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(TurretIOInputs inputs) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRadPerSec) {}

  public default void runPosition(double positionRad) {}

  public default void setPosition(double positionRad) {}

  public default void zeroPosition() {}

  /** Set gain constants */
  public default void configureGains(double kP, double kI, double kD, double kS, double kV) {}

  /** Set gain constants */
  public default void configureGains(
      double kP, double kI, double kD, double kS, double kV, double kA) {}
}
