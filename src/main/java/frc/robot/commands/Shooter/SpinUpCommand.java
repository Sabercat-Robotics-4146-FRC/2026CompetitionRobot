package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;

public class SpinUpCommand extends Command {

  private final Shooter shooter;

  public SpinUpCommand(Shooter s) {
    this.shooter = s;
  }

  @Override
  public void execute() {
    shooter.setShooterState(ShooterState.SHOOTING);
  }

  @Override
  public void end(boolean interrupted) {}
}
