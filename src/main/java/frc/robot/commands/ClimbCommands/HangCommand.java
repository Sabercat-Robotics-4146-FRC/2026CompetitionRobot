package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class HangCommand extends Command{

  private final Climb climb;

  public HangCommand(Climb climb){
    this.climb = climb; 
  }

  @Override
  public void execute(){
    climb.extendClimb();
  }

  @Override
  public boolean isFinished(){
    return climb.isAtHangedPosition(); 
  }

}
