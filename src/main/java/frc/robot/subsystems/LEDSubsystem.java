package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private Color teamColor;

    private static LEDSubsystem instance = null;

    private LEDSubsystem(int ledPort, int ledLength) {
        // PWM port 9
        // Must be a PWM header, not MXP or DIO
        this.led = new AddressableLED(ledPort);
        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        this.ledBuffer = new AddressableLEDBuffer(ledLength);
        this.led.setLength(ledBuffer.getLength());
        this.teamColor = DriverStation.getAlliance().equals(Alliance.Red) ? new Color(255, 0, 0) : new Color(0, 0, 255);
        this.setSolidRGB(0, 255, 0);

        // Set the data
        this.led.setData(ledBuffer);

        this.led.start();
        // scheduler = new Light_Scheduler();
    }

    public static LEDSubsystem getInstance() {
        if (instance == null) {
            instance = new LEDSubsystem(LEDConstants.LED_PORT, LEDConstants.LED_LENGTH);
        }

        return instance;
    }

    // ========================================================
    // ==================== SOLID COLOR =======================

    /**
     * Sets the color of all LEDs to the specified RGB color
     * 
     * @param R Red value (0-255)
     * @param G Green value (0-255)
     * @param B Blue value (0-255)
     */
    public void setSolidRGB(int R, int G, int B) {
        for (var i = 0; i < this.ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values
            this.ledBuffer.setRGB(i, R, G, B);
        }
    }

    // ========================================================
    // ===================== FLASHING =========================

    private boolean flashing = false;
    private int counter = 0;

    /**
     * Flashes the LEDs between on and off. Should be called periodically. Speed is
     * meant to be controlled by the scheduler
     * 
     * @param R           Red value (0-255)
     * @param G           Green value (0-255)
     * @param B           Blue value (0-255)
     * @param pulseLength how long each pulse and gap is. (1 pulse is 0.02 seconds)
     */
    public void flashingRGB(int R, int G, int B, double pulseLength) {
        counter++;

        if (!this.flashing) {
            setSolidRGB(R, G, B);
        } else {
            setSolidRGB(0, 0, 0);
        }

        if (counter % pulseLength == 0) {
            this.flashing = !this.flashing;
            counter = 0; // avoids overflow
        }
    }

    // ========================================================
    // ===================== RAINBOW ==========================

    private int rainbowHueValue = 0;

    /**
     * Creates a rainbow using the LEDs (must be called periodically for cool
     * effect)
     * This is also the example code from the documentation:
     * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html#creating-a-rainbow-effect
     * 
     * @param speed_multiplier The speed multiplier of the rainbow. <strong>This is
     *                         how you speed up the light effect.</strong>
     */
    public void rainbow(int speed_multiplier) {
        final var length = ledBuffer.getLength();
        for (var i = 0; i < length; i++) {
            final var hue = (this.rainbowHueValue + (i * 180 / length)) % 180;
            this.ledBuffer.setHSV(i, hue, 255, 255);
        }
        this.rainbowHueValue += 1 * speed_multiplier;
        this.rainbowHueValue %= 180;
    }

    // ========================================================
    // ====================== CYCLON ==========================

    private int cylon_center = 0;
    private double cylon_velocity = 1;

    /**
     * Creates a cylon effect using the LEDs (must be called periodically for the
     * effect).
     * 
     * @param hue              The hue of the cylon from [0-180]
     * @param saturation       The saturation of the cylon from [0-255]
     * @param speed_multiplier The speed multiplier of the cylon. <em>This is
     *                         how you speed up the light effect.</em>
     */
    public void cylon(int hue, int saturation, int speed_multiplier) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            int value = 255 - (Math.abs(this.cylon_center - i) * 50);
            if (value < 0) {
                value = 0;
            }
            this.ledBuffer.setHSV(i, hue, saturation, value);
        }
        this.cylon_center += (cylon_velocity * speed_multiplier);
        if (this.cylon_center >= this.ledBuffer.getLength() || this.cylon_center < 0) {
            this.cylon_velocity *= -1;
        } // makes the cylon go back and forth
    }

    /**
     * Creates a cylon effect but with two opposite things...
     * 
     * @param hue              The hue of the cylon from [0-180]
     * @param saturation       The saturation of the cylon from [0-255]
     * @param speed_multiplier The speed multiplier of the cylon. <em>This is how
     *                         you speed up the light effect.</em>
     * @param hue2             The hue of the second cylon from [0-180]
     * @param sat2             The saturation of the second cylon from [0-255]
     */
    public void cylon_but_two(int hue, int saturation, int speed_multiplier, int hue2, int sat2) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            int value = 255 - (Math.abs(this.cylon_center - i) * 40);
            this.ledBuffer.setHSV(i, hue, saturation, value);
            if (value < 0) {
                value = 0;
                int value2 = 255
                        - (Math.abs((ledBuffer.getLength() - this.cylon_center) % ledBuffer.getLength() - i) * 40);
                if (value2 < 0) {
                    value2 = 0;
                }
                this.ledBuffer.setHSV(i, hue2, sat2, value2);
            } // this is a bit wierd but it should be faster to compute than before (at least
              // double speed)
        }
        this.cylon_center += (cylon_velocity * speed_multiplier);
        if (this.cylon_center >= this.ledBuffer.getLength() || this.cylon_center < 0) {
            this.cylon_velocity *= -1;
        } // makes the cylon go back and forth
    }

    // ========================================================
    // ===================== CARNIVAL =========================

    private int carnival_index = 0;

    /**
     * Creates a carnival effect using the LEDs (must be called periodically).
     * Looks best if the number of colors times the segment length is divisible by
     * the number of LEDs (150).
     * 
     * @param colors         An array of {@link edu.wpi.first.wpilibj.util.Color}s
     *                       to cycle through
     * @param segment_length The length of each segment of the color
     */
    public void carnival(Color[] colors, int segment_length) {
        int current_color_index = 0;
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED((i + carnival_index) % ledBuffer.getLength(), colors[current_color_index]);
            if ((i + 1) % segment_length == 0) {
                current_color_index++;
                current_color_index %= colors.length;
            }
        }
        carnival_index++;
    }

    @Override
    public void periodic() {
        this.led.setData(ledBuffer);
    }
}