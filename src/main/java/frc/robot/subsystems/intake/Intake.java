package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;

public class Intake {
  private final IntakeIO io;
  private final DigitalInput limitSwitchExtended = new DigitalInput(0); //true when hopper is extended
  private final DigitalInput limitSwitchRetracted = new DigitalInput(1);
  private final Debouncer debouncerOne = new Debouncer(0.05);
  private final Debouncer debouncerTwo = new Debouncer(0.05);

  public Intake(IntakeIO io) {
    this.io = io;
    io.setMode();
    io.setExtenderMode(false);
  }

  // returns true if limit switch 1 is not being held
  public boolean isExtended() {
    return debouncerOne.calculate(limitSwitchExtended.get()) && !debouncerTwo.calculate(limitSwitchRetracted.get());
  }

  // return true if limit switch is being held
  public boolean isRetracted() {
    return debouncerTwo.calculate(limitSwitchRetracted.get()) && !debouncerOne.calculate(limitSwitchExtended.get());
  }

  // false if limit switch is pressed, true if not pressed
  public void extendIntake() {
      if(isRetracted()){
      io.setOutputExtender(0.8); // tune this number
      }
      else{
        io.stopExtender(); 
        io.setExtenderMode(true); 
      } 
  }

  public void stopExtender(){
    io.stopExtender();
  }

  // only run intake if roller is down
  public void runIntake() {
    if (isExtended()) {
      io.setOutputRoller();
    }
  }

  public void stopIntake() {
    io.stopRoller();
  }

  public void retractIntake() {
    io.stopRoller();
    if (isRetracted()) {
      io.setOutputExtender(-0.8); // tune this number
    }
  }

  public void periodic(){
    System.out.println("Limit Switch Retracted Value" + limitSwitchRetracted.get());
    System.out.println("Limit Switch Extended Value" + limitSwitchExtended.get());
  }
}
