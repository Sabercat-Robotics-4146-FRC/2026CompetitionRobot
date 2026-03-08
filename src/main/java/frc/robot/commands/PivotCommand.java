package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class PivotCommand extends Command {
  private final Intake intake;
  private boolean pivotState;

  public PivotCommand(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    pivotState = intake.isRetracted();
  }

  @Override
  public void execute() {
    if (pivotState) {
      intake.extendIntake();

    } else {
      intake.retractIntake();
    }
  }

  @Override
  public boolean isFinished() {
    if (pivotState) {
      return intake.isExtended(); // we started retracted, finish when extended
    } else {
      return intake.isRetracted(); // we started extended, finish when retracted
    }
    // return pivotState ? intake.isExtended() : intake.isRetracted();
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopExtender();
  }
}
