// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.kicker;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Kicker extends RBSISubsystem {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  /** Creates a new Kicker. */
  public Kicker(KickerIO io) {
    this.io = io;

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
    Logger.processInputs("Kicker", inputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec);

    // Log Kicker setpoint
    Logger.recordOutput("Kicker/SetpointRPM", velocityRPM);
  }

  /** Stops the Kicker. */
  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput(key = "Mechanism/Kicker")
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
