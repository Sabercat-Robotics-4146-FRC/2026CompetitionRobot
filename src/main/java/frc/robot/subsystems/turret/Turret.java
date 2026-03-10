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

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
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

  public enum TurretState {
    AUTO,
    LEFT,
    RIGHT,
    CENTER,
    HOMING
  }

  public TurretState state = TurretState.AUTO;

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
  private static final double kMechSign = -1.0;

  private static Pose3d hubTarget;

  // Limit switches
  @AutoLogOutput
  private final DigitalInput leftLimitSwitch = new DigitalInput(kLeftLimitDIOChannel);

  @AutoLogOutput
  private final DigitalInput rightLimitSwitch = new DigitalInput(kRightLimitDIOChannel);

  @AutoLogOutput private boolean homing = false;
  @AutoLogOutput private boolean homed = false;

  // Lead compensation inputs (field-relative)
private Supplier<Double> fieldVxMps;
private Supplier<Double> fieldVyMps;

// Time-of-flight  (seconds)
private static final double kShotTofSec = 0.30; 
private static final double kMaxLeadMeters = 0.75; // cap compensation

  /** Creates a new turret. */
  public Turret(TurretIO io, Supplier<Double> fieldVxSupplier, Supplier<Double> fieldVySupplier) {
    this.io = io;
    this.fieldVxMps = fieldVxSupplier;
    this.fieldVyMps = fieldVySupplier;

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

    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      hubTarget = kHubTargetRed;
    } else {
      hubTarget = kHubTargetBlue;
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

    var result = turretCam.getLatestResult();
    var visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
    boolean multiTag = visionEst.isPresent();
    if (visionEst.isEmpty()) {
      visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
    }

    boolean visionGood = false;
    if (visionEst.isPresent()) {
      if (multiTag) {
        visionGood = true;
      } else if (result.hasTargets()) {
        double amb = result.getBestTarget().getPoseAmbiguity();
        visionGood = amb >= 0.0 && amb < 0.15; // tune
      }
    }

    if (visionEst.isEmpty() || !visionGood) {
      // TODO: Search or use robot odometry
      System.out.println("No turret pose estimate available.");
    } else {
      Logger.recordOutput("Turret/TurretPose", visionEst.get().estimatedPose);
      if (state == TurretState.AUTO) {
        aimAtTarget(
            visionEst.get().estimatedPose,
            hubTarget,
            !leftLimitSwitch.get(),
            !rightLimitSwitch.get());
      }
    }

    if (state == TurretState.RIGHT) {
      runPosition(1.48);
    } else if (state == TurretState.LEFT) {
      runPosition(-1.35);
    } else if (state == TurretState.CENTER) {
      runPosition(0.0);
    }

    Logger.recordOutput("Turret/State", state);
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
              setState(TurretState.HOMING);
            },
            this),
        Commands.repeatingSequence(Commands.runOnce(() -> runVolts(0.3)))
            .until(() -> !rightLimitSwitch.get())
            .andThen(
                () -> {
                  io.stop();
                  io.setPosition(Math.PI / 2);
                  homed = true;
                  homing = false;
                },
                this)
            .andThen(
                () -> {
                  runPosition(0.0);
                  setState(TurretState.AUTO);
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
    // 1) Desired field yaw to target
    double dx = targetField.getX() - turretPoseField.getX();
    double dy = targetField.getY() - turretPoseField.getY();
    double desiredFieldYaw = Math.atan2(dy, dx);

    // 2) Current turret field yaw (from vision pose)
    double turretFieldYaw = turretPoseField.getRotation().getZ();

    // 3) Current turret mechanical angle (radians)
    double currentMechRad = inputs.positionRad;

    // 4) Field yaw at mechanical zero
    double fieldYawAtZeroNow = MathUtil.angleModulus(turretFieldYaw - kMechSign * currentMechRad);

    // 5) Desired mechanical setpoint
    double desiredMechRad = kMechSign * MathUtil.angleModulus(desiredFieldYaw - fieldYawAtZeroNow);

    // 6) Clamp to range
    double clampedMechRad = MathUtil.clamp(desiredMechRad, kMinCmdRad, kMaxCmdRad);

    // 7) Limit switch interlock
    if (leftLimitPressed && clampedMechRad < currentMechRad) clampedMechRad = currentMechRad;
    if (rightLimitPressed && clampedMechRad > currentMechRad) clampedMechRad = currentMechRad;

    lastSetpointRad = clampedMechRad;
    Logger.recordOutput("Turret/lastSetpointRad", lastSetpointRad);
    Logger.recordOutput("Turret/currentMechRad", currentMechRad);
    Logger.recordOutput("Turret/turretFieldYaw", turretFieldYaw);
    Logger.recordOutput("Turret/fieldYawAtZeroNow", fieldYawAtZeroNow);
    Logger.recordOutput("Turret/desiredFieldYaw", desiredFieldYaw);
    Logger.recordOutput("Turret/desiredMechRad", desiredMechRad);

    runPosition(lastSetpointRad);
  }

  public void aimAtTargetCompensated(
    Pose3d turretPoseField,
    Pose3d targetField,
    boolean leftLimitPressed,
    boolean rightLimitPressed) {

  // --- Lead compensation ---
  double leadX = fieldVxMps.get() * kShotTofSec;
  double leadY = fieldVyMps.get() * kShotTofSec;

  // cap lead magnitude
  double leadMag = Math.hypot(leadX, leadY);
  if (leadMag > kMaxLeadMeters) {
    double scale = kMaxLeadMeters / leadMag;
    leadX *= scale;
    leadY *= scale;
  }

  double futureTurretX = turretPoseField.getX() + leadX;
  double futureTurretY = turretPoseField.getY() + leadY;

  // 1) Desired field yaw to target (from predicted turret position)
  double dx = targetField.getX() - futureTurretX;
  double dy = targetField.getY() - futureTurretY;
  double desiredFieldYaw = Math.atan2(dy, dx);

  // 2) Current turret field yaw (from vision pose)
  double turretFieldYaw = turretPoseField.getRotation().getZ();

  // 3) Current turret mechanical angle (radians)
  double currentMechRad = inputs.positionRad;

  // 4) Field yaw at mechanical zero
  double fieldYawAtZeroNow = MathUtil.angleModulus(turretFieldYaw - kMechSign * currentMechRad);

  // 5) Desired mechanical setpoint
  double desiredMechRad = kMechSign * MathUtil.angleModulus(desiredFieldYaw - fieldYawAtZeroNow);

  // 6) Clamp to range
  double clampedMechRad = MathUtil.clamp(desiredMechRad, kMinCmdRad, kMaxCmdRad);

  // 7) Limit switch interlock
  if (leftLimitPressed && clampedMechRad < currentMechRad) clampedMechRad = currentMechRad;
  if (rightLimitPressed && clampedMechRad > currentMechRad) clampedMechRad = currentMechRad;

  lastSetpointRad = clampedMechRad;

  Logger.recordOutput("Turret/lastSetpointRad", lastSetpointRad);
  Logger.recordOutput("Turret/leadX", leadX);
  Logger.recordOutput("Turret/leadY", leadY);
  Logger.recordOutput("Turret/futureTurretX", futureTurretX);
  Logger.recordOutput("Turret/futureTurretY", futureTurretY);

  runPosition(lastSetpointRad);
}

  public void setHoming(boolean state) {
    homing = state;
  }

  public void setState(TurretState state) {
    this.state = state;
  }
}
