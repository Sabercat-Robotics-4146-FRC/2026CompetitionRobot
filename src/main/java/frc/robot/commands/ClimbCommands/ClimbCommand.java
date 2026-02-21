package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class ClimbCommand extends Command {

  private final Climb climb;

  public ClimbCommand(Climb climb) {
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
