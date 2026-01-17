package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

// MConstants.java
public class MConstants {
    // Shooter PID
    public static final PIDFCoefficients shooterCoeff = new PIDFCoefficients(0.01d, 0.00d, 0.0d, 0.00052d);

    // ========================= Intake =========================
    public static final double intakePowerIn = -1.0;   // pulls balls in
    public static final double intakePowerOut = 1.0;   // pushes balls out

    // ========================= Indexer =========================
    public static final double indexerPowerIn = -1.0;   // feeds balls forward
    public static final double indexerPowerOut = 1.0; // clears area / reverse

    // ========================= Ejector =========================
    public static final double ejectorDown = 0.0;      // rest position
    public static final double ejectorUp = 1.0;       // up position

    // ========================= Outtake Flap ====================
    public static final double flapDown = 0.7;
    public static final double flapUp = 0.3;

    // ========================= Outtake ==========================
    public static final double SHOOTER_P = 0;
    public static final double SHOOTER_KV = 0;
    public static final double SHOOTER_KS = 0;
}

