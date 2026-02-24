package frc.robot.subsystems.LEDS;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDS extends SubsystemBase{

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer; 

  private enum LEDState { 
    GOLD, 
    BLUE_BLINK,
    RED_BLINK }
  private LEDState currentState = LEDState.GOLD;

  public LEDS(){
    led = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(60);
    led.setLength(buffer.getLength());
    led.setData(buffer);
    led.start();
  }

  public void setState(LEDState state) {
    this.currentState = state;
  }

  public void setGold(){
   LEDPattern base = LEDPattern.solid(new Color(0.83, 0.69, 0.22));;
   base.applyTo(buffer);
   led.setData(buffer);
    
  }

  public void setBlueBlinking(){
   LEDPattern base = LEDPattern.solid(new Color(0, 0, 0.5));
   LEDPattern pattern = base.blink(Seconds.of(0.5));
   pattern.applyTo(buffer);
   led.setData(buffer);

  }

  public void setRedBlinking(){
    LEDPattern base = LEDPattern.solid(new Color(0.6, 0.0, 0.08));
   LEDPattern pattern = base.blink(Seconds.of(0.5));
   pattern.applyTo(buffer);
   led.setData(buffer);

  }

  public void chooseBlinkingColors(){
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if(alliance == Alliance.Blue){
      setState(LEDState.BLUE_BLINK);
    }
    else if (alliance == Alliance.Red){
      setState(LEDState.RED_BLINK);
    }
  }



  @Override
 public void periodic() {
    switch (currentState) {
        case GOLD -> setGold();
        case BLUE_BLINK -> setBlueBlinking();
        case RED_BLINK -> setRedBlinking();
    }
}

}
