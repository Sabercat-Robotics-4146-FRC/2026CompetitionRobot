package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class RunIntakeCommand extends Command{
  private final Intake intake;

  public RunIntakeCommand(Intake intake){
    this.intake = intake; 

  }

  public void execute(){
    intake.runIntake(); 
  }

}
