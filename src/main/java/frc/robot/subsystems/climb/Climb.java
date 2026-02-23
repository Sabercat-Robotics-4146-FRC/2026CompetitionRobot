package frc.robot.subsystems.climb;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotDevices;

public class Climb extends SubsystemBase {

  private DigitalInput limitSwitch = new DigitalInput(RobotDevices.CLIMB_LIMIT_SWITCH);
  private Debouncer debouncer = new Debouncer(0.1);
  private ClimbIOTalonFX motor;
  private boolean isHomed = false;

  // constructor for climb
  public Climb(ClimbIOTalonFX motor) {
    this.motor = motor;
  }

  // retract climb all the way down
  public boolean isHomed() {
    boolean isHomed = !debouncer.calculate(limitSwitch.get());
    return isHomed;
  }

  // after homing, set brake mode to hold position
  public void setBrakeMode() {
    motor.setMode(true);
  }

  // release brake mode for extending climb
  public void setCoastMode() {
    motor.setMode(false);
  }

  // home the climb
  public void homeClimb() {
    setCoastMode();
    if (!isHomed) {
      motor.goHome(-0.5);
    } else {
      motor.stop();
    }
    motor.zeroPosition();
    setBrakeMode();
    System.out.println("position" + motor.getPosition());
  }

  // release climb all the way up
  public void extendClimb() {
    setCoastMode();
    motor.goUp();
    setBrakeMode();
    System.out.println("position" + motor.getPosition());
  }

  public boolean isAtHangedPosition() {
    return motor.getPosition() == ClimbIOTalonFX.hangedPosition;
  }
}
