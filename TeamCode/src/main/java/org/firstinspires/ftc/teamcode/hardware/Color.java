package org.firstinspires.ftc.teamcode.hardware;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

/**
 * Class to represent the bytes in an RGB or RGBW color.
 */
public class Color {
    public static final Color BLACK = new Color((byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00);
    public static final Color WHITE = new Color((byte) 0x0F, (byte) 0x0F, (byte) 0x0F, (byte) 0x08);
    public static final Color RED = new Color((byte) 0x08, (byte) 0x00, (byte) 0x00, (byte) 0x00);

    public static final Color ORANGE = new Color((byte) 0x06, (byte) 0x01, (byte) 0x00, (byte) 0x00);
    public static final Color YELLOW = new Color((byte) 0x0B, (byte) 0x08, (byte) 0x00, (byte) 0x00);
    public static final Color GREEN = new Color((byte) 0x00, (byte) 0x08, (byte) 0x00, (byte) 0x00);
    public static final Color BLUE = new Color((byte) 0x00, (byte) 0x00, (byte) 0x08, (byte) 0x00);

    public static final Color PURPLE = new Color((byte) 0x08, (byte) 0x00, (byte) 0x08, (byte) 0x00);
    public static final Color PINK = new Color((byte) 0x08, (byte) 0x00, (byte) 0x03, (byte) 0x00);
    public final byte red;
    public final byte green;
    public final byte blue;
    public final byte white;

    // MY COLORS
    // Define full brightness constants
    private static final Color FULL_RED   = new Color((byte) 0xFF, (byte) 0x00, (byte) 0x00, (byte) 0x01);
    private static final Color FULL_GREEN = new Color((byte) 0x00, (byte) 0xFF, (byte) 0x00, (byte) 0x01);
    private static final Color FULL_BLUE  = new Color((byte) 0x00, (byte) 0x00, (byte) 0xFF, (byte) 0x01);

    public static final Color brightRED   = Color.scale(FULL_RED, 0.1);
    public static final Color brightGreen = Color.scale(FULL_GREEN, 0.1);
    public static final Color brightBlue  = Color.scale(FULL_BLUE, 0.1);

    /**
     * Initializes a Color instance with RGBW values.
     *
     * @param red   The red value
     * @param green The green value
     * @param blue  The blue value
     * @param white The white value
     */
    public Color(byte red, byte green, byte blue, byte white) {
        this.red = red;
        this.green = green;
        this.blue = blue;
        this.white = white;
    }

    /**
     * Overloaded constructor to only take RGB values.
     *
     * @param red   The red value
     * @param green The green value
     * @param blue  The blue value
     */
    public Color(byte red, byte green, byte blue) {
        this(red, green, blue, (byte) 0x00);
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Color) {
            Color other = (Color) obj;
            return red == other.red
                    && green == other.green
                    && blue == other.blue
                    && white == other.white;
        }
        return false;
    }

    @NonNull
    @SuppressLint("DefaultLocale")
    @Override
    public String toString() {
        return String.format("Color(red=%d, green=%d, blue=%d, white=%d)", red, green, blue, white);
    }

    /**
     * Returns a new Color scaled by a factor (0.0 to 1.0)
     */
    public static Color scale(Color c, double factor) {
        return new Color(
                clamp((c.red & 0xFF) * factor),
                clamp((c.green & 0xFF) * factor),
                clamp((c.blue & 0xFF) * factor),
                clamp((c.white & 0xFF) * factor)
        );
    }

    private static byte clamp(double val) {
        // Clamp between 0 and 255, then cast
        int clamped = (int) Math.max(0, Math.min(255, val));
        return (byte) clamped;
    }
}