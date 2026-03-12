package frc.robot.commands.Composition;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Intake.PivotCommand;
import frc.robot.commands.Shooter.SpinUpCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;

public class AutoShoot extends SequentialCommandGroup{

   public AutoShoot(Kicker kicker, Shooter shooter, Intake intake) {
    addCommands(
        new ParallelCommandGroup
        (new PivotCommand(intake), 
        new SpinUpCommand(shooter)),
        new WaitUntilCommand(() -> shooter.isAtSetpoint()),
        new RunCommand(() -> kicker.runVolts(4), kicker)     
        .withTimeout(4.0));
  }

}
