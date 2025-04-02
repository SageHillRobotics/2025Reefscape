package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;

    private final int LENGTH = 49;
    private final int PWM_PORT = 9;

    public LED(){
        m_led = new AddressableLED(PWM_PORT);
        m_ledBuffer = new AddressableLEDBuffer(LENGTH);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.start();
        
        setDefaultCommand(solidGreen());
    }

    public void setGreen(){
        LEDPattern green = LEDPattern.solid(Color.kGreen);
        green.applyTo(m_ledBuffer);
        // m_led.setData(m_ledBuffer);
    }

    public void setBlinkGreen(){
        LEDPattern green = LEDPattern.solid(Color.kGreen);
        LEDPattern pattern = green.blink(Units.Seconds.of(0.1));
        pattern.applyTo(m_ledBuffer);
        // m_led.setData(m_ledBuffer);
    }

    public void setBlinkPurple(){
        LEDPattern purple = LEDPattern.solid(Color.kPurple);
        LEDPattern pattern = purple.blink(Units.Seconds.of(0.1));
        pattern.applyTo(m_ledBuffer);
        // m_led.setData(m_ledBuffer);
    }
    public void setBlinkRed(){
        LEDPattern purple = LEDPattern.solid(Color.kRed);
        LEDPattern pattern = purple.blink(Units.Seconds.of(0.1));
        pattern.applyTo(m_ledBuffer);
    }

    public void setBlinkOrange(){
        LEDPattern purple = LEDPattern.solid(Color.kOrange);
        LEDPattern pattern = purple.blink(Units.Seconds.of(0.1));
        pattern.applyTo(m_ledBuffer);
    }

    public Command setOff(){
        return (run(() -> LEDPattern.solid(Color.kBlack).applyTo(m_ledBuffer)));
    }

    public Command blinkGreen(){
        return run(() -> setBlinkGreen());

    }

    public Command blinkPurple(){
        return run(() -> setBlinkPurple());

    }

    public Command blinkRed(){
        return run(() -> setBlinkRed());
    }

    public Command blinkOrange(){
        return run(() -> setBlinkOrange());
    }

    public Command solidGreen(){
        return run(() -> setGreen());
    }

    @Override
    public void periodic() {
        m_led.setData(m_ledBuffer);
    }
}
