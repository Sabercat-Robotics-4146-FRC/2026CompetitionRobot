
 package frc.robot.subsystems.climb;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotDevices;
import java.util.function.DoubleSupplier;

public class Climb extends SubsystemBase {

  private DigitalInput limitSwitch = new DigitalInput(RobotDevices.CLIMB_LIMIT_SWITCH);
  private Debouncer debouncer = new Debouncer(0.05);
  private ClimbIO motor;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  public double homedPosition = 0;
  public static double hangedPosition = 15;
  private DoubleSupplier lt;
  private DoubleSupplier rt;

  // constructor for climb
  public Climb(ClimbIO motor, DoubleSupplier lefttrigger, DoubleSupplier righttrigger) {
    this.motor = motor;
    this.rt = righttrigger;
    this.lt = lefttrigger;
  }

  // retract climb all the way down
  public boolean isHomed() {
    return debouncer.calculate(limitSwitch.get());
  }

  public boolean isAtHangedPosition() {
    return Math.abs(inputs.positionRad - hangedPosition) < 2;
  }

  public boolean isAtHomedPosition() {
    return Math.abs(inputs.positionRad - 5) < 1;
  }

  public void zeroPosition() {
    motor.zeroPosition();
  }

  public void setBrakeMode() {
    motor.setMode(true);
  }

  public void setCoastMode() {
    motor.setMode(false);
  }

  // home the climb
  public void homeClimb() {
    motor.goHome();
  }

  // release climb all the way up
  public void extendClimb() {
    motor.goUp();
  }

  public void stopMotor() {
    motor.stop();
  }

  public void periodic() {
    motor.updateInputs(inputs);
    System.out.println("position" + inputs.positionRad);

    motor.setVoltage((lt.getAsDouble() - rt.getAsDouble()) * 9);
  }
}
