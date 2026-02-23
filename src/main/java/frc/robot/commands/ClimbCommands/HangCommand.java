package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climb.Climb;

public class HangCommand extends SequentialCommandGroup{
  public HangCommand(RobotContainer container, Climb climb){
    addCommands(
      new ExtendCommand(climb),
      //drive to correct position command
      new RetractCommand(climb)
    );
  }

}
