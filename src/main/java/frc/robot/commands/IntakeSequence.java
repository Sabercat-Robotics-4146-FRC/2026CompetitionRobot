package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake;

public class IntakeSequence extends SequentialCommandGroup{
  public IntakeSequence(RobotContainer container, Intake intake){
    addCommands(
      new ExtendIntakeCommand(intake),
      new RunIntake(intake)

    );
  }

}
