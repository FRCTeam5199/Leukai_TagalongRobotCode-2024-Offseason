package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IndexerSubsystem;

public class LEDSubsystem extends SubsystemBase {
    private static LEDSubsystem ledSubsystem;

    public enum LEDMode {
        SHUTTLING,
        AMPTRAP,
        SHOOTING,
        REACHEDSHOOTINGSPEED,
    }

    public boolean isIntaking = false;
    public boolean hasNote = false;

    public LEDMode selectedLEDMode;
    AddressableLED LEDLights = new AddressableLED(Constants.LEDs.LED_PORT);
    AddressableLEDBuffer LEDBuffer = new AddressableLEDBuffer(Constants.LEDs.LED_LENGTH);

    public static LEDSubsystem getInstance() {
        if (ledSubsystem == null) {
            ledSubsystem = new LEDSubsystem();
        }
        return ledSubsystem;
    }

    private LEDSubsystem() {
    }

    public void init() {
        LEDLights.setLength(LEDBuffer.getLength());
        selectedLEDMode = null;
    }

    public void start() {
        LEDLights.start();
    }

    public void stop() {
        LEDLights.stop();
    }

    public void setMode(LEDMode mode) {
        this.selectedLEDMode = mode;
    }

    @Override
    public void periodic() {
        if (isIntaking) setColor(Color.kRed);
        else if (hasNote) setColor(Color.kGreen);
        else setColor(Color.kBlue);

        if (!IndexerSubsystem.getInstance().isNoteInIndexer() && !IndexerSubsystem.getInstance().isNoteInAmpTrap())
            hasNote = false;

        LEDLights.setData(LEDBuffer);
    }

    public int getRed(int LEDindex) {
        return LEDBuffer.getRed(LEDindex);
    }

    public int getGreen(int LEDindex) {
        return LEDBuffer.getGreen(LEDindex);
    }

    public int getBlue(int LEDindex) {
        return LEDBuffer.getBlue(LEDindex);
    }

    public void setRGB(int index, int red, int blue, int green) {
        LEDBuffer.setRGB(index, red, blue, green);
    }

    public void setRGB(int red, int blue, int green) {
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
            LEDBuffer.setRGB(i, red, blue, green);
        }
    }

    public void setColor(int index, Color color) {
        LEDBuffer.setLED(index, color);
    }

    public void setColor(Color color) {
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
            LEDBuffer.setLED(i, color);
        }
    }

    public void setRed(int index, int red) {
        LEDBuffer.setRGB(index, red, LEDBuffer.getGreen(index), LEDBuffer.getBlue(index));
    }

    public void setRed(int red) {
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
            LEDBuffer.setRGB(i, red, LEDBuffer.getGreen(i), LEDBuffer.getBlue(i));
        }
    }

    public void setGreen(int index, int green) {
        LEDBuffer.setRGB(index, LEDBuffer.getRed(index), green, LEDBuffer.getBlue(index));
    }

    public void setGreen(int green) {
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
            LEDBuffer.setRGB(i, LEDBuffer.getRed(i), green, LEDBuffer.getBlue(i));
        }
    }

    public void setBlue(int index, int blue) {
        LEDBuffer.setRGB(index, LEDBuffer.getRed(index), LEDBuffer.getGreen(index), blue);
    }

    public void setBlue(int blue) {
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
            LEDBuffer.setRGB(i, LEDBuffer.getRed(i), LEDBuffer.getGreen(i), blue);
        }
    }

    public void setShift(int red1, int green1, int blue1, int red2, int green2, int blue2) {
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
            if (i <= (LEDBuffer.getLength() / 2)) {
                LEDBuffer.setRGB(i, red1, green1, blue1);
            } else {
                LEDBuffer.setRGB(i, red1, green1, blue1);
            }
        }
    }

    public void setShift(Color color1, Color color2) {
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
            if (i <= (LEDBuffer.getLength() / 2)) {
                LEDBuffer.setLED(i, color1);
            } else {
                LEDBuffer.setLED(i, color2);
            }
        }
    }

    public void setShift(Color... colors) {
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
            LEDBuffer.setLED(i, colors[(int) Math.round(LEDBuffer.getLength() / colors.length)]);
        }
    }

    public void setFade(int red, int green, int blue) {
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
            LEDBuffer.setRGB(i, red + i, green + i, blue + i);
        }
    }

    public void setFade(int red, int green, int blue, double strength) {
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
            LEDBuffer.setRGB(i, red + (int) Math.round(i * strength), green + (int) Math.round(i * strength), blue + (int) Math.round(i * strength));
        }
    }

    public void setFade(Color... color) {
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
            LEDBuffer.setLED(i, color[Math.round(LEDBuffer.getLength() / color.length)]);
        }
    }

    public void rainbow() {
        int rainbowFirstPixelHue = 0;

        // For every pixel
        for (var i = 0; i < LEDBuffer.getLength(); i++) {
            LEDBuffer.setHSV(i, (rainbowFirstPixelHue + (i * 180 / LEDBuffer.getLength())) % 180, 255, 128);
        }

        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;
    }

}