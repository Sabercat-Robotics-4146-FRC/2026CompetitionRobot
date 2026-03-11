package frc.robot.commands.Kicker;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;

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
   public void end(boolean interrupted) {
   }
  
}
