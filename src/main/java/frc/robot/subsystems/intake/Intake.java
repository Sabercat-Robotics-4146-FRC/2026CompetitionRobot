package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final IntakeIOTalonFX io;
  private final DigitalInput limitSwitchExtended = new DigitalInput(2);
  private final DigitalInput limitSwitchRetracted = new DigitalInput(3);
  private final Debouncer debouncerOne = new Debouncer(0.05);
  private final Debouncer debouncerTwo = new Debouncer(0.05);

  public Intake(IntakeIOTalonFX io) {
    this.io = io;
    io.setMode();
    io.setExtenderMode(false);
  }

  // returns true if retracted
  public boolean isExtended() {
    return !debouncerOne.calculate(limitSwitchExtended.get());
  }

  // return true if limit switch is being held
  public boolean isRetracted() {
    return !debouncerTwo.calculate(limitSwitchRetracted.get());
  }

  public void extendIntake() {
    io.setExtender();
  }

  // only run intake if roller is down
  public void runIntake() {

    io.setOutputRoller();
  }

  public void stopIntake() {
    io.stopRoller();
  }

  public void stopExtender() {
    io.stopExtender();
  }

  public void retractIntake() {
    io.setRetraction();
  }

  @Override
  public void periodic() {

    // System.out.println("Limit Switch Retracted" + isRetracted());
    // System.out.println("Limit Switch Extended" + isExtended());
  }
}
