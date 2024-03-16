package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LEDState;

import static frc.robot.Constants.LEDConstants.LED_LENGTH;
import static frc.robot.RobotMap.LedMap.LED_PWM_PORT;

public class LedSubsystem extends SubsystemBase {
    public enum Effect{
        FILL,CYCLE,FLASH,CUSTOM
    }
    private Effect effect = Effect.CUSTOM;

    private final AddressableLED led = new AddressableLED(LED_PWM_PORT);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LED_LENGTH);
    private int rainbowFirstPixelHue;
    private final Timer timer = new Timer();

    public LedSubsystem(){
        led.setLength(ledBuffer.getLength());
        led.start();
    }
    public void write() {
        led.setData(ledBuffer);
    }

    public void close() {
        led.close();
    }

    public void setRGB(int index,Color color){
        effect = Effect.CUSTOM;
        ledBuffer.setLED(index,color);
    }

    public void setRGB(int index,int r,int g,int b){
        effect = Effect.CUSTOM;
        ledBuffer.setRGB(index,r,g,b);
    }
    public void setRGB(int index, LEDState LEDState){
        effect = Effect.CUSTOM;
        ledBuffer.setRGB(index, LEDState.r, LEDState.g, LEDState.b);
    }


    public void fillRGB(Color color){
        effect = Effect.FILL;
        for(int i=0;i<ledBuffer.getLength();i++){
            setRGB(i, color);
        }
    }

    public void fillRGB(int r,int g,int b){
        effect = Effect.FILL;
        for(int i=0;i< ledBuffer.getLength();i++){
            setRGB(i,r,g,b);
        }
    }
    public void fillRGB(LEDState LEDState){
        effect = Effect.FILL;
        for(int i=0;i< ledBuffer.getLength();i++){
            setRGB(i, LEDState.r, LEDState.g, LEDState.b);
        }
    }
    public void setLEDState(int first,int last,LEDState LEDState){
        for(int i=first;i< last;i++){
            setRGB(i, LEDState.r, LEDState.g, LEDState.b);
        }
    }
    public void blinkRGB(LEDState color1, LEDState color2, double timestamp,double duration){
        effect = Effect.FLASH;
        if((int)(timestamp/duration)%2 == 0){
            setLEDState(0, ledBuffer.getLength(),color1);
        }
        else {
            setLEDState(0, ledBuffer.getLength(),color2);
        }

    }




    public void rainbow() {
        // For every pixel
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
            // Set the value
            ledBuffer.setHSV(i, rainbowFirstPixelHue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 1;
        // Check bounds
        rainbowFirstPixelHue %= 180;
    }





}

