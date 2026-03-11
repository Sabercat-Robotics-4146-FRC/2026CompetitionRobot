package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends RBSISubsystem {
  private final IntakeIO io;
  private final DigitalInput limitSwitchExtended = new DigitalInput(2);
  private final DigitalInput limitSwitchRetracted = new DigitalInput(3);
  private final Debouncer debouncerOne = new Debouncer(0.05);
  private final Debouncer debouncerTwo = new Debouncer(0.05);
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
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
  protected void rbsiPeriodic() {
    io.updateInputsRoller(inputs);
     io.updateInputsExtender(inputs);
    Logger.processInputs("Intake", inputs);
  }
}
