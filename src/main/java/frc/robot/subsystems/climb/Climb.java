package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotDevices;

public class Climb extends SubsystemBase {

  private DigitalInput limitSwitch = new DigitalInput(RobotDevices.CLIMB_LIMIT_SWITCH);
  private ClimbIOTalonFX motor;
  private boolean isHomed = false;

  // constructor for climb
  public Climb(ClimbIOTalonFX motor) {
    this.motor = motor;
  }

  // retract climb all the way down
  public boolean isHomed() {
    boolean isHomed = limitSwitch.get();
    return isHomed;
  }

  // home the climb
  public void homeClimb() {
    if (!isHomed) {
      motor.goHome(-0.5);
    }
  }

  // after homing, set brake mode to hold position
  public void setMode() {
    motor.setMode(true);
  }

  // release brake mode for extending climb
  public void setCoastMode() {
    motor.setMode(false);
  }

  // release climb all the way up
  public void extendClimb() {
    motor.goUp();
  }
}
