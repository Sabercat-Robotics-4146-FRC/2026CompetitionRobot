package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase{


  // positions of the climb, in inches from limit switch 
  public enum Position{
    HOMED(0),
    HIT(10);

    private final double inches; 

    private Position(double inches){
      this.inches = inches; 
    }

  }
  
  private ClimbIOTalonFX motor; 
  private boolean isHomed = false; 
  private Position maxPosition;
  private Position currentPosition; 
  // constructor for climb 
  public Climb(){
    motor = new ClimbIOTalonFX(); 

  }


  public void setPosition(Position position){

  }

  public void setPercentOutput(double percent){

  }

  public boolean isHomed(){
    return isHomed; 
  }
  

  

}
