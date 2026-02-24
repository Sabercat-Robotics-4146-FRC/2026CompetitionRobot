package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDS.LEDS;

public class LightCommand extends Command{
  private final LEDS led;

  public LightCommand(LEDS led){
    this.led = led;
  }

  public void execute(){
    led.chooseBlinkingColors();
  }

   @Override
    public boolean isFinished() {
        return true; // or false for continuous behavior
    }

}

//put this in climb sequence