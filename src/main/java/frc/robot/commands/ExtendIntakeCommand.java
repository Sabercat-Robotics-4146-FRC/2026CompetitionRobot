/*
package frc.robot.commands;


public class ExtendIntakeCommand extends Command {
  private final Intake intake;


  public ExtendIntakeCommand(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void execute() {
    if(intake.isRetracted()){
       intake.extendIntake();
    }
  }

  @Override
  public boolean isFinished(){
    return intake.isExtended();

  }

  @Override
  public void end(boolean interrupted){
    intake.stopExtender();
    intake.setExtenderMode(true);

  }
}
*/
