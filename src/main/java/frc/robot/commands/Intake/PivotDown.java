package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class PivotDown extends Command {
  private final Intake intake;

  public PivotDown(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    if (intake.isRetracted()) {
      intake.extendIntake();
    }
  }

  @Override
  public boolean isFinished() {
    return intake.isExtended();
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopExtender();
  }
}
