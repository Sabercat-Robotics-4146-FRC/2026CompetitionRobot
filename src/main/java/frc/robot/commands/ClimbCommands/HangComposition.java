package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climb.Climb;

public class HangComposition extends SequentialCommandGroup {
  public HangComposition(RobotContainer container, Climb climb) {
    addCommands(
        new ExtendCommand(climb),
        // new AutoPilotCommands(new AutoPIlot)
        new RetractCommand(climb));
  }
}
