package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class RetractCommand extends Command{
 private final Intake intake;


  public RetractCommand(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void execute() {
    if(intake.isExtended()){
       intake.retractIntake();
    }
  }

  @Override
  public boolean isFinished(){
    return intake.isRetracted();

  }

  @Override 
  public void end(boolean interrupted){
    intake.stopExtender();
    intake.setExtenderMode(true);

  }
}
