package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.RBSISubsystem;

public class Intake{
  private final IntakeIO io;
  private final DigitalInput limitSwitchExtended = new DigitalInput(0);
  private final DigitalInput limitSwitchRetracted = new DigitalInput(1);
  private final Debouncer debouncerOne = new Debouncer(0.05);
  private final Debouncer debouncerTwo = new Debouncer(0.05);

  public Intake(IntakeIO io){
    this.io = io; 
    io.setMode();
  }


  //returns true if limit switch is pressed 
  private boolean limitSwitchExtended(){
    return !debouncerOne.calculate(limitSwitchExtended.get());
  }

  private boolean limitSwitchRetracted(){
    return !debouncerTwo.calculate(limitSwitchRetracted.get());
  }

  //returns true if both limit switches are pressed 
  public boolean isExtended(){
    return limitSwitchExtended(); 
  }

  public boolean isRetracted(){
    return limitSwitchRetracted();
  }

  //false if limit switch is pressed, true if not pressed
  public void extendIntake(){
    if (!isExtended()) {
    io.setPercentOutputExtender(0.8); //tune this number
    }
    else{
      io.stopExtender();
    }
    io.setMode();
  }

  //only run intake if roller is down 
  public void runIntake(){
    if(isExtended()){
    io.setPercentOutputRoller(0.8);
    }
  }

  public void stopIntake(){
    io.stopRoller();
  }

  public void retractIntake(){
    io.stopRoller();
    if(isRetracted()){
      io.setPercentOutputExtender(-0.8); //tune this number
    }
  }

 

}
