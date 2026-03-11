package frc.robot.commands.Kicker;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.kicker.Kicker;

public class KickCommand extends Command {

  private final Kicker kicker;

  public KickCommand(Kicker k) {
    this.kicker = k;
  }

  @Override
  public void execute() {
    kicker.runVolts(3);
  }

  @Override
  public void end(boolean interrupted) {}
}
