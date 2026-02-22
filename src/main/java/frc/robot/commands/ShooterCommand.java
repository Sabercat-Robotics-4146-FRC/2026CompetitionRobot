package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;

public class ShooterCommand extends Command{

  private final Shooter shooter;

  public ShooterCommand(Shooter shooter){
    this.shooter = shooter; 
  }

  public void execute(){
    shooter.setVelocityRPS(); 
  }

  public void end(boolean interrupted){
    shooter.stopShooter(); 
  }

}
