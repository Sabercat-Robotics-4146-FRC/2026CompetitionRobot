package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final IntakeIOTalonFX io;
  private final DigitalInput limitSwitchExtended =
      new DigitalInput(3); // true when hopper is extended
  private final DigitalInput limitSwitchRetracted = new DigitalInput(2);
  private final Debouncer debouncerOne = new Debouncer(0.05);
  private final Debouncer debouncerTwo = new Debouncer(0.05);

  public Intake(IntakeIOTalonFX io) {
    this.io = io;
    io.setMode();
    io.setExtenderMode(false);
  }

  // returns true if limit switch 1 is not being held
  public boolean isExtended() {
    return !debouncerOne.calculate(limitSwitchExtended.get())
        && debouncerTwo.calculate(limitSwitchRetracted.get());
  }

  // return true if limit switch is being held
  public boolean isRetracted() {
    return debouncerTwo.calculate(limitSwitchRetracted.get())
        && !debouncerOne.calculate(limitSwitchExtended.get());
  }

  // false if limit switch is pressed, true if not pressed
  public void extendIntake() {
    io.setExtender(); // tune this number
  }

  // only run intake if roller is down
  public void runIntake() {
    io.setOutputRoller();
  }

  public void stopIntake() {
    io.stopRoller();
  }

  public void stopExtender() {
    io.stopExtender();
    io.setExtenderMode(false);
  }

  public void retractIntake() {
    io.setRetraction();
  }

  @Override
  public void periodic() {
    // extendIntake();
    // System.out.println("Limit Switch Retracted" + limitSwitchRetracted.get());
    // System.out.println("Limit Switch Extended" + limitSwitchExtended.get());
    // System.out.println("angle" + io.getPosition());
  }
}
