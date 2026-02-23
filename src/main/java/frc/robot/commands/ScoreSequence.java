package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;


//first rotate turret to direct shooter through cameras 
public class ScoreSequence extends SequentialCommandGroup{

  public ScoreSequence(RobotContainer robotContainer, Shooter shooter, Kicker kicker){
    addCommands(
      new ShooterCommand(shooter),
      new WaitCommand(0.5),
      new KickerCommand(kicker)
    );
  }
  

}
