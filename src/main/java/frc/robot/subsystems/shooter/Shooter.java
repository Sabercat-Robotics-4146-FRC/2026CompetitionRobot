package frc.robot.subsystems.shooter;

public class Shooter {
  private final ShooterIOTalonFX shooter;

  public Shooter(ShooterIOTalonFX shooter) {
    this.shooter = shooter;
  }

  public void setVelocityRPS() {
    shooter.setVelocityRPS();
  }

  public void stopShooter() {
    shooter.stop();
  }
}
