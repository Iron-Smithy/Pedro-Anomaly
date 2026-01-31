package org.firstinspires.ftc.teamcode.pedroPathing.Teleop;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.MConstants;
import org.firstinspires.ftc.teamcode.tools.Actions.TurretAction;

import java.util.Map;
import java.util.TreeMap;

/**
 * Goal pose, distance→RPM map, and heading PID for auto-aim. Turn output uses
 * KP*error + KI*integral + KD*derivative for fast, smooth turn with less oscillation.
 */
public class ShooterAimHelper {
    private final Pose goalPose;
    private final double closeDistanceInches;
    private final TreeMap<Double, Long> speedMap;

    // Heading PID state
    private double integral = 0;
    private double lastError = Double.NaN;
    private long lastTimeMs = 0;

    public ShooterAimHelper(Pose goalPose, double closeDistanceInches, TreeMap<Double, Long> speedMap) {
        this.goalPose = goalPose;
        this.closeDistanceInches = closeDistanceInches;
        this.speedMap = speedMap;
    }

    public static ShooterAimHelper defaultRed() {
        TreeMap<Double, Long> map = new TreeMap<>();
        map.put(12.0 * 1, 850L);
        map.put(12.0 * 5, 1130L);
        map.put(12.0 * 9, 1200L);
        map.put(12.0 * 12, 1640L);
        map.put(12.0 * 14, 1680L);
        return new ShooterAimHelper(new Pose(132, 136, 0), 8 * 12, map);
    }
    // public static ShooterAimHelper defaultBlue() { currently copy of red, uncomment and tune
    //     TreeMap<Double, Long> map = new TreeMap<>();
    //     map.put(12.0 * 1, 850L);
    //     map.put(12.0 * 5, 1130L);
    //     map.put(12.0 * 9, 1200L);
    //     map.put(12.0 * 12, 1640L);
    //     map.put(12.0 * 14, 1680L);
    //     return new ShooterAimHelper(new Pose(132, 136, 0), 8 * 12, map);
    // }

    public double getDistanceToGoal(Pose currentPose) {
        return Math.hypot(goalPose.getX() - currentPose.getX(), goalPose.getY() - currentPose.getY());
    }

    /**
     * Turn correction for auto-aim using heading PID (KP, KI, KD from MConstants).
     * Call resetHeadingPID() when driver releases aim to clear integral.
     */
    public double getTurnCorrection(Pose currentPose) {
        double targetAngle = Math.atan2(
                goalPose.getY() - currentPose.getY(),
                goalPose.getX() - currentPose.getX());
        targetAngle += Math.PI;
        double error = angleWrap(targetAngle - currentPose.getHeading());

        long now = System.currentTimeMillis();
        double dt = (lastTimeMs == 0) ? 0.02 : (now - lastTimeMs) / 1000.0;
        lastTimeMs = now;

        integral += error * dt;
        integral = Math.max(-MConstants.AIM_MAX_INTEGRAL, Math.min(MConstants.AIM_MAX_INTEGRAL, integral));

        double derivative = Double.isNaN(lastError) ? 0 : (error - lastError) / Math.max(dt, 1e-6);
        lastError = error;

        double out = MConstants.AIM_KP * error
                + MConstants.AIM_KI * integral
                + MConstants.AIM_KD * derivative;
        out = Math.max(-MConstants.AIM_MAX_OUTPUT, Math.min(MConstants.AIM_MAX_OUTPUT, out));
        return out;
    }

    /** Call when driver stops holding auto-aim so integral doesn’t carry over. */
    public void resetHeadingPID() {
        integral = 0;
        lastError = Double.NaN;
        lastTimeMs = 0;
    }

    /** Interpolated shooter TPS for current distance to goal. */
    public long getTargetSpeed(Pose currentPose) {
        double dist = getDistanceToGoal(currentPose);
        if (dist <= speedMap.firstKey()) return speedMap.firstEntry().getValue();
        if (dist >= speedMap.lastKey()) return speedMap.lastEntry().getValue();
        Map.Entry<Double, Long> low = speedMap.floorEntry(dist);
        Map.Entry<Double, Long> high = speedMap.ceilingEntry(dist);
        if (low == null || high == null) return 2000;
        double fraction = (dist - low.getKey()) / (high.getKey() - low.getKey());
        return (long) (low.getValue() + fraction * (high.getValue() - low.getValue()));
    }

    public Pose getGoalPose() { return goalPose; }
    public TreeMap<Double, Long> getSpeedMap() { return speedMap; }

    public static double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}
