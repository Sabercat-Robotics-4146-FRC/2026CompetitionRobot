package frc.robot.commands.Composition;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.PivotDown;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommand extends SequentialCommandGroup {
  public IntakeCommand(Intake intake) {
    addCommands(new PivotDown(intake), new RunIntake(intake));
  }
}
