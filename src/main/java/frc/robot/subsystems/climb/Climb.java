package frc.robot.subsystems.climb;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotDevices;

public class Climb extends SubsystemBase {

  private DigitalInput limitSwitch = new DigitalInput(RobotDevices.CLIMB_LIMIT_SWITCH);
  private Debouncer debouncer = new Debouncer(0.05);
  private ClimbIO motor;
  private boolean isHomed = false;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  public double homedPosition = 0;
  public static double hangedPosition = 100;

  // constructor for climb
  public Climb(ClimbIO motor) {
    this.motor = motor;
  }

  // retract climb all the way down
  public boolean isHomed() {
    isHomed = debouncer.calculate(limitSwitch.get());
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
    motor.goHome();
  }

  public boolean isAtHangedPosition() {
    if (inputs.positionRad > hangedPosition - 4 && inputs.positionRad < hangedPosition + 4) {
      return true;
    } else {
      return false;
    }
  }

  // release climb all the way up
  public void extendClimb() {
    // setCoastMode();
    motor.goUp();
    motor.setPosition();

    // letBrakeMode();
    // System.out.println("position" + motor.getPosition());
  }

  public void stopMotor(){
    motor.stop();
  }

  public void periodic() {
    motor.updateInputs(inputs);
    System.out.println("limit switch value" + limitSwitch.get());
    System.out.println("position" + inputs.positionRad);
  }
}
