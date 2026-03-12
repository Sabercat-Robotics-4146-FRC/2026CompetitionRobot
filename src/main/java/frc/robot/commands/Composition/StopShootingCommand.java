package frc.robot.commands.Composition;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;

public class StopShootingCommand extends SequentialCommandGroup {

  public StopShootingCommand(Kicker kicker, Shooter shooter) {
    addCommands(
        Commands.runOnce(() -> kicker.runVolts(0), kicker),
        new WaitCommand(0.25),
        Commands.runOnce(() -> shooter.setShooterState(ShooterState.IDLE), shooter));
  }
}
