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
        map.put(12.0, 850L); //good
        map.put(60.0, 1130L); //good
        map.put(72.0, 1200L); //good
        map.put(84.0, 1250L); //good
        map.put(96.0, 1370L); //good
        map.put(108.0, 1450L); //maybe
        map.put(120.0, 1460L); //maybe
        map.put(132.0, 1470L);
        map.put(144.0, 1480L);
        map.put(168.0, 1490L);
        return new ShooterAimHelper(new Pose(132, 136, 0), 8 * 12, map);
    }
    public static ShooterAimHelper defaultBlue() {
        TreeMap<Double, Long> map = new TreeMap<>();
        map.put(12.0, 850L); //decent
        map.put(60.0, 1110L); //GOOD
        map.put(72.0, 1160L); //GOOD
        map.put(84.0, 1230L); //GOOD // values copied from red side table
        map.put(96.0, 1300L); //decent
        map.put(108.0, 1450L); //maybe
        map.put(120.0, 1460L); //maybe
        map.put(132.0, 1470L);
        map.put(144.0, 1480L);
        map.put(168.0, 1490L);
        return new ShooterAimHelper(new Pose(132, 136, 0).mirror(), 8 * 12, map);
    }

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
