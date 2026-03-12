package frc.robot.commands.Kicker;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.kicker.Kicker;

public class KickCommand extends Command {

  private final Kicker kicker;

  public KickCommand(Kicker kicker) {
    this.kicker = kicker;
    addRequirements(kicker);
  }

  @Override
  public void execute() {
    kicker.runVolts(4);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
