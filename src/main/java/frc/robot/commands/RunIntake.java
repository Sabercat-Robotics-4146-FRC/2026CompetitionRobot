package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class RunIntake extends Command {

  private final Intake intake;

  public RunIntake(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void execute() {
    if (intake.isExtended()) {
      intake.runIntake();
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }
}
