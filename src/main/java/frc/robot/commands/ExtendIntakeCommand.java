package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class ExtendIntakeCommand extends Command {
  private final Intake intake;

  public ExtendIntakeCommand(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void execute() {
    intake.retractIntake();
    System.out.println("first test");
  }

  @Override
  public boolean isFinished() {
    return intake.isRetracted();
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopExtender();
    // intake.setExtenderMode(true);

  }
}
