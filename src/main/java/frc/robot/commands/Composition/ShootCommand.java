package frc.robot.commands.Composition;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Kicker.KickCommand;
import frc.robot.commands.Shooter.SpinUpCommand;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;

public class ShootCommand extends SequentialCommandGroup {

  public ShootCommand(Kicker kicker, Shooter shooter) {
    addCommands(
        Commands.runOnce(() -> kicker.runVolts(-2), kicker),
        new WaitCommand(0.25),
        Commands.runOnce(
            () -> {
              kicker.stop();
            },
            kicker),
        new SpinUpCommand(shooter),
        new WaitUntilCommand(() -> shooter.isAtSetpoint()),
        new KickCommand(kicker));
  }
}
