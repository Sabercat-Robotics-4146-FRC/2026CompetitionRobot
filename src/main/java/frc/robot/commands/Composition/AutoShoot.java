package frc.robot.commands.Composition;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.PivotCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

public class AutoShoot extends SequentialCommandGroup {

  public AutoShoot(Kicker kicker, Shooter shooter, Intake intake, Turret turret) {
    addCommands(
        Commands.runOnce(() -> turret.Home(), turret),
        new ParallelCommandGroup(new PivotCommand(intake), new ShootCommand(kicker, shooter)),
        new WaitCommand(5),
        new StopShootingCommand(kicker, shooter));
  }
}
