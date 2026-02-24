package frc.robot.subsystems.kicker;

public class Kicker {
  private final KickerIOTalonFX kicker;

  public Kicker(KickerIOTalonFX kicker) {
    this.kicker = kicker;
  }

  public void setVoltage() {
    kicker.setVoltage(6);
  }

  public void stop() {
    kicker.stop();
  }
}
