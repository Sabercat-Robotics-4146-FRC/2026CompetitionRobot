package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;


//add intake must be retracted before climbing 
public class RetractCommand extends Command {

  private final Climb climb;

  public RetractCommand(Climb climb) {
    this.climb = climb;
  }

  @Override
  public void execute() {
    climb.homeClimb();
  }

  @Override
  public boolean isFinished() {
    return climb.isHomed();
  }
}
