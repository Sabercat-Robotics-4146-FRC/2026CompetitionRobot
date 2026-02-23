package frc.robot.subsystems.kicker;

public class Kicker {
  private final KickerIO kicker; 

  public Kicker(KickerIO kicker){
    this.kicker = kicker; 
  }

  public void setVoltage(){
    kicker.setVoltage(6); 
  }

   public void stop(){
    kicker.stop();
  }

}
