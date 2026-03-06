// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.turret;

import static frc.robot.Constants.TurretConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class Turret extends RBSISubsystem {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  // For Aiming
  private static final double kMinRad = -Math.PI / 2.0;
  private static final double kMaxRad = +Math.PI / 2.0;
  private static final double kStopMarginRad = Math.toRadians(2.0);
  private static final double kMinCmdRad = kMinRad + kStopMarginRad;
  private static final double kMaxCmdRad = kMaxRad - kStopMarginRad;
  private double lastSetpointRad = 0.0;
  public static final AprilTagFieldLayout kTagLayout = FieldConstants.aprilTagLayout;
  private final PhotonPoseEstimator photonEstimator =
      new PhotonPoseEstimator(kTagLayout, kTurretToCam);
  private final PhotonCamera turretCam = new PhotonCamera(kTurretCamName);

  // Limit switches
  @AutoLogOutput
  private final DigitalInput leftLimitSwitch = new DigitalInput(kLeftLimitDIOChannel);

  @AutoLogOutput
  private final DigitalInput rightLimitSwitch = new DigitalInput(kRightLimitDIOChannel);

  @AutoLogOutput private boolean homing = false;
  @AutoLogOutput private boolean homed = false;

  /** Creates a new turret. */
  public Turret(TurretIO io) {
    this.io = io;

    switch (Constants.getMode()) {
      case REAL:
        io.configureGains(240.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        break;
      case REPLAY:
        io.configureGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        break;
      case SIM:
      default:
        io.configureGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        break;
    }

    // if (!homed){
    //   Home().schedule();
    // }
  }

  /** Periodic function -- inherits timing logic from RBSISubsystem */
  @Override
  protected void rbsiPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    // TODO: left limit behavior checking
    // if (leftLimitSwitch.get()) {
    //   io.setPosition(-Math.PI / 2);
    // }

    if (!rightLimitSwitch.get()) {
      io.setPosition(Math.PI / 2);
    }

    var visionEst = photonEstimator.estimateCoprocMultiTagPose(turretCam.getLatestResult());
    if (visionEst.isEmpty()) {
      visionEst = photonEstimator.estimateLowestAmbiguityPose(turretCam.getLatestResult());
    }

    if (visionEst.isEmpty()) {
      // TODO: Search or use robot odometry
      System.out.println("No turret pose estimate available.");
    } else {
      aimAtTarget(
          visionEst.get().estimatedPose,
          kHubTarget,
          !leftLimitSwitch.get(),
          !rightLimitSwitch.get());
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

    // Log turret setpoint
    Logger.recordOutput("Turret/SetpointRPM", velocityRPM);
  }

  public void runPosition(double positionRad) {
    io.runPosition(positionRad);

    Logger.recordOutput("Turret/SetpointRad", positionRad);
  }

  public void zeroPosition() {
    io.zeroPosition();
  }

  public Command Home() {
    return new SequentialCommandGroup(
        Commands.runOnce(
            () -> {
              homing = true;
              homed = false;
            },
            this),
        Commands.repeatingSequence(Commands.runOnce(() -> runVolts(0.3)))
            .until(() -> !rightLimitSwitch.get())
            .andThen(
                () -> {
                  homed = true;
                  homing = false;
                  io.stop();
                },
                this)
            .andThen(
                () -> {
                  runPosition(0.0);
                }));
  }

  /** Stops the Turret. */
  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }

  public void aimAtTarget(
      Pose3d turretPoseField,
      Pose3d targetField,
      boolean leftLimitPressed,
      boolean rightLimitPressed) {
    if (turretPoseField != null) {
      Pose3d turretPose = turretPoseField;

      // 1) Desired field yaw to target
      double dx = targetField.getX() - turretPose.getX();
      double dy = targetField.getY() - turretPose.getY();
      double desiredFieldYaw = Math.atan2(dy, dx);

      // 2) Current turret field yaw (from vision pose)
      double turretFieldYaw = turretPose.getRotation().getZ(); // radians

      // 3) Current turret mechanical angle (from encoder, what MotionMagic uses)
      double currentMechRad = inputs.positionRad;

      // 4) Compute field yaw at mechanical zero *this cycle*:
      //    psi0 = psi_t - theta
      double fieldYawAtZeroNow = MathUtil.angleModulus(turretFieldYaw - currentMechRad);

      // 5) Desired mechanical angle:
      //    theta_des = wrap(psi_des - psi0)
      double desiredMechRad = MathUtil.angleModulus(desiredFieldYaw - fieldYawAtZeroNow);

      // 6) Clamp to 180° turret
      double clampedMechRad = MathUtil.clamp(desiredMechRad, kMinCmdRad, kMaxCmdRad);

      // 7) Limit switch safety (only allow away from the pressed switch)
      if (leftLimitPressed && clampedMechRad < currentMechRad) clampedMechRad = currentMechRad;
      if (rightLimitPressed && clampedMechRad > currentMechRad) clampedMechRad = currentMechRad;

      lastSetpointRad = clampedMechRad;
      Logger.recordOutput("Turret/lastSetpointRad", lastSetpointRad);
    }

    // 8) Motion Magic position setpoint is the mechanical radians
    // runPosition(lastSetpointRad);
    System.out.println("Turret aiming at: " + lastSetpointRad);
  }

  public void setHoming(boolean state) {
    homing = state;
  }
}
