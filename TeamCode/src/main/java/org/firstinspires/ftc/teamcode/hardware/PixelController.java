// EjectorAction.java
package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Auton.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.MConstants;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.AdafruitNeoPixel;
import org.firstinspires.ftc.teamcode.hardware.Color;

import java.util.Arrays;

public class PixelController {
    public final AdafruitNeoPixel pixel;

    public Color[] PixColor = new Color[9];
    public Color[] AllianceColor = new Color[9];

    public PixelController(Alliance alliance) {
        if (alliance == Alliance.RED) {
            Arrays.fill(AllianceColor, Color.brightRED);
        } else {
            Arrays.fill(AllianceColor, Color.brightBlue);
        }
        pixel = RobotHardware.pixel;

        pixel.initialize(9, 3);

        try { Thread.sleep(50); } catch (InterruptedException ignored) {}

        pixel.clearLeds();
        pixel.show();
    }

    public void ballSensorTele(int ballCount, boolean ball_fire_ready) {
        // Initialize a 9-LED strip with all LEDs off (Black)
        Color[] strip = new Color[9];
        for (int i = 0; i < 9; i++) {
            strip[i] = Color.BLACK;
        }

        Color baseColor = ball_fire_ready ? Color.brightGreen  : Color.brightBlue;

        Color altColor = ball_fire_ready ? Color.brightLime : Color.brightCyan;

        if (ballCount == 1) {
            // Outermost LEDs on both sides (Index 0 and 8)
            strip[0] = Color.brightBlue;
            strip[8] = Color.brightBlue;
        }
        else if (ballCount == 2) {
            // Outer 3 most on both sides (0,1,2 and 6,7,8)
            strip[0] = Color.brightRED; strip[1] = Color.brightRED; strip[2] = Color.brightRED;
            strip[6] = Color.brightRED; strip[7] = Color.brightRED; strip[8] = Color.brightRED;
        }
        else if (ballCount >= 3) {
            // All 9 LEDs full
            Color[] rainbow = {
                    Color.brightGreen, Color.brightGreen, Color.brightGreen,
                    Color.brightGreen, Color.brightGreen, Color.brightGreen,
                    Color.brightGreen, Color.brightGreen, Color.brightGreen
            };
            strip = rainbow;
        }

        // Send the final array to the controller
        setColors(strip);
    }


    public void setColors(Color[] color) {
        PixColor = color;
        pixel.setLeds(0, color);
        show();
    }

    public void show() {
        pixel.show();
    }
    public void setAllianceColor() {
        setColors(AllianceColor);
    }
}
