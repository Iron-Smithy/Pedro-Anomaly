package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
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

    // ========================= Ejector =========================
    public static final double blockerIn = 0.5;      // rest position
    public static final double blockerOut = 0.2;       // up position

    // ========================= Outtake Flap ====================
    public static final double flapDown = 0.7;
    public static final double flapUp = 0.3;
    public static final Double distToOpen = 41.0;

    // ========================= Outtake (velocity PID: power = KV*vel + P*(velErr) + KS) =========================
    /** Feedforward: power per unit TPS. Tune so motor reaches target without overshoot. */
    public static final double SHOOTER_KV = 0.00038; // 0.00052
    /** Proportional on velocity error. Reduces oscillation, improves lock.*/
    public static final double SHOOTER_P = 0.012; // 0.012
    /**  Static friction / minimum power to move. */
    public static final double SHOOTER_KS = 0.0455; // 0.06

    // ========================= Turret (velocity PID: power = KV*vel + P*(velErr) + KS) =========================
    public static final com.pedropathing.control.PIDFCoefficients turretPIDFCoefficent =
            new com.pedropathing.control.PIDFCoefficients(0.006, 0, 0.0002, 0);
    public static final double MAGIC = 183.90558;

    // ========================= Auto-aim heading PID =========================
    public static final double AIM_KP = 0.08;
    public static final double AIM_KI = 0.002;
    public static final double AIM_KD = 0.012;
    /** Max turn power magnitude (0..1). */
    public static final double AIM_MAX_OUTPUT = 0.85;
    /** Anti-windup: cap integral contribution. */
    public static final double AIM_MAX_INTEGRAL = 0.25;

    // ========================= Ball sensors (Rev Color V3 distance) =========================
    /** Distance (cm) below which a ball is considered present. Tune per sensor. */
    public static final double[] BALL_DETECT_DISTANCE_CM = {2, 6, 2.7, 3};

    // ========================= POSE =========================
    public static final Pose startPoseRed = new Pose(124, 92, Math.toRadians(0));
    public static final Pose goalResetPoseRed = new Pose(118, 127, Math.toRadians(40));
    public static final Pose humanPlayerPoseRed = new Pose(8.75, 13, 0);
    public static final Pose goalPoseRed = new Pose(132, 136, 0);
}

