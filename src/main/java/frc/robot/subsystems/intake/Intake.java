package frc.robot.subsystems.intake;

import frc.robot.util.RBSISubsystem;

public class Intake{
  private final IntakeIO io;

  public Intake(IntakeIO io){
    this.io = io; 

  }

  public void extendIntake(){
    io.setPercentOutputExtender(0.8);
  }

  public void runIntake(){
    io.setMode();
    io.setPercentOutputRoller(0.8);
  }

 

}
