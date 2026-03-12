package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;

public class SpinUpCommand extends Command {

  private final Shooter shooter;

  public SpinUpCommand(Shooter shooter) {
    this.shooter = shooter;
  }

  @Override
  public void execute() {
    shooter.setShooterState(ShooterState.SHOOTING);
  }

  @Override
  public boolean isFinished() {
    return shooter.isAtSetpoint();
  }
}
