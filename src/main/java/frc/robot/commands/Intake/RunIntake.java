package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class RunIntake extends Command {

  private final Intake intake;

  public RunIntake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.runIntake();
    System.out.println("first");
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
