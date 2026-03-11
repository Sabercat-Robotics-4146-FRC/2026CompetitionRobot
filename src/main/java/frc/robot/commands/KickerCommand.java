package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.kicker.Kicker;

public class KickerCommand extends Command {

  private final Kicker kicker;

  public KickerCommand(Kicker kicker) {
    this.kicker = kicker;
  }

  public void execute() {}

  public void end(boolean interrupted) {
    kicker.stop();
  }
}
