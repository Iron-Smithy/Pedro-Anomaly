package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.AdafruitNeoPixel;
import org.firstinspires.ftc.teamcode.hardware.Color;

import java.util.Arrays;

@TeleOp(name = "LED Test", group = "Tests")
public class LEDTest extends LinearOpMode {
    enum LEDMode {
        BARBERSHOP,
        SECTIONS,
        BLINK,
        PULSE,
        JOY
    }

    LEDMode currentMode = LEDMode.BARBERSHOP;

    AdafruitNeoPixel pixel;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware.init(hardwareMap);

        pixel = RobotHardware.pixel;

        pixel.initialize(9, 3);
        pixel.clearLeds();
        pixel.show();

        waitForStart();

        while (opModeIsActive()) {
            // 1. INPUT: Check for button presses to switch modes
            if (gamepad1.triangle)      currentMode = LEDMode.BARBERSHOP;
            else if (gamepad1.circle)   currentMode = LEDMode.SECTIONS;
            else if (gamepad1.square)   currentMode = LEDMode.BLINK;
            else if (gamepad1.cross)    currentMode = LEDMode.PULSE;
            else if (gamepad1.ps)    currentMode = LEDMode.JOY;

            // 2. LOGIC: Run the code for the selected mode
            switch (currentMode) {
                case BARBERSHOP:
                    // Scrolling White, Red, White, Blue
                    Color[] pole = {Color.WHITE, Color.RED, Color.WHITE, Color.BLUE};
                    int offset = (int) (getRuntime() * 8); // Speed multiplier
                    for (int i = 0; i < 9; i++) {
                        pixel.setLeds(i, pole[(i + offset) % pole.length]);
                    }
                    break;

                case SECTIONS:
                    // Fixed RGB blocks
                    for (int i = 0; i < 3; i++) pixel.setLeds(i, Color.RED);
                    for (int i = 3; i < 6; i++) pixel.setLeds(i, Color.GREEN);
                    for (int i = 6; i < 9; i++) pixel.setLeds(i, Color.BLUE);
                    break;

                case BLINK:
                    // Toggle all LEDs on/off every 500ms
                    boolean isOn = ((int) (getRuntime() * 2) % 2) == 0;
                    for (int i = 0; i < 9; i++) {
                        pixel.setLeds(i, isOn ? Color.YELLOW : Color.BLACK);
                    }
                    break;

                case PULSE:
                    double sin = Math.sin(getRuntime() * 4);
                    int brightness = (int) ((sin * 127.5) + 127.5);
                    byte byteBrightness = (byte) brightness;

                    // We only create a new color and update if the brightness
                    // has changed significantly. This saves I2C bandwidth.
                    Color pulseColor = new Color(byteBrightness, (byte) 0, byteBrightness);

                    for (int i = 0; i < 9; i++) {
                        pixel.setLeds(i, pulseColor);
                    }
                    break;

                case JOY:
                    // Convert -1.0 to 1.0 into 0 to 255
                    int r = (int) (gamepad1.left_stick_x * 127.5 + 127.5);
                    int g = (int) (gamepad1.left_stick_y * 127.5 + 127.5);
                    int b = (int) (gamepad1.right_stick_x * 127.5 + 127.5);

                    // Cast those ints to bytes for the Color constructor
                    Color joyColor = new Color((byte) r, (byte) g, (byte) b);

                    for (int i = 0; i < 9; i++) {
                        pixel.setLeds(i, joyColor);
                    }
                    break;
            }

            // 3. OUTPUT: Send the buffer to the strip
            pixel.show();
            idle();
            sleep(1000/10); // 10x per second
        }

        pixel.clearLeds();
        pixel.show();
    }
}