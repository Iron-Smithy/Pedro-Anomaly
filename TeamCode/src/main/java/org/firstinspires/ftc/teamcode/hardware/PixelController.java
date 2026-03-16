// EjectorAction.java
package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.LinkedList;

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

    // List to store ball counts with timestamps
    private final LinkedList<BallDetection> detectionHistory = new LinkedList<>();

    // Simple helper class to track history
    private static class BallDetection {
        int count;
        long timestamp;
        BallDetection(int count, long timestamp) {
            this.count = count;
            this.timestamp = timestamp;
        }
    }

    public void ballSensorTele(int ballCount, boolean ball_fire_ready) {
        long currentTime = System.currentTimeMillis();

        // 1. Add current reading to history
        detectionHistory.add(new BallDetection(ballCount, currentTime));

        // 2. Remove any readings older than 1 seconds (1000ms)
        while (!detectionHistory.isEmpty() && (currentTime - detectionHistory.peekFirst().timestamp > 1000)) {
            detectionHistory.removeFirst();
        }

        // 3. Find the highest ball count in the current 2-second window
        int maxBallCount = 0;
        for (BallDetection entry : detectionHistory) {
            if (entry.count > maxBallCount) {
                maxBallCount = entry.count;
            }
        }

        // Use 'maxBallCount' for the display logic instead of the raw 'ballCount'
        Color[] strip = new Color[9];
        Arrays.fill(strip, Color.BLACK);

        if (maxBallCount == 1) {
            strip[0] = Color.brightBlue;
            strip[8] = Color.brightBlue;
        }
        else if (maxBallCount == 2) {
            strip[0] = Color.brightRED; strip[1] = Color.brightRED; strip[2] = Color.brightRED;
            strip[6] = Color.brightRED; strip[7] = Color.brightRED; strip[8] = Color.brightRED;
        }
        else if (maxBallCount >= 3) {
            Arrays.fill(strip, Color.brightGreen);
        }

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
