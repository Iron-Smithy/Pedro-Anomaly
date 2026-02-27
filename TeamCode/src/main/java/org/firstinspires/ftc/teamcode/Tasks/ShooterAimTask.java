package org.firstinspires.ftc.teamcode.Tasks;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.MConstants;

import java.util.Map;
import java.util.TreeMap;

/**
 * Goal pose, distance→RPM map, and heading PID for auto-aim. Turn output uses
 * KP*error + KI*integral + KD*derivative for fast, smooth turn with less oscillation.
 */
public class ShooterAimTask {
    private final Pose goalPose; // location of the thing we're shooting at
    private final TreeMap<Double, Long> speedMap; // the correct motor speeds to shoot a goal at set distances

    // Heading PID state
    private double integral = 0;
    private double lastError = Double.NaN;
    private long lastTimeMs = 0;

    public ShooterAimTask(Pose goalPose, TreeMap<Double, Long> speedMap) {
        this.goalPose = goalPose;
        this.speedMap = speedMap;
    }

    public static TreeMap<Double, Long> speedMap() {
        TreeMap<Double, Long> map = new TreeMap<>();
        map.put(12.0, 850L);
        map.put(30.0, 960L); //open
        map.put(40.0, 960L); //open
        map.put(50.0, 1030L);
        map.put(60.0, 1080L);
        map.put(70.0, 1110L);
        map.put(80.0, 1170L);
        map.put(90.0, 1250L);
        map.put(100.0, 1280L);
        map.put(110.0, 1310L);
        map.put(115.0, 1330L);
        map.put(125.0, 1350L);
        map.put(130.0, 1400L);
        map.put(135.0, 1430L);
        map.put(140.0, 1460L);
        map.put(145.0, 1490L);
        map.put(150.0, 1550L);

        return map;
    }

    public static ShooterAimTask defaultRed() { // Deprecated
        TreeMap<Double, Long> map = new TreeMap<>();
        map.put(12.0, 850L);
        map.put(30.0, 960L); //open
        map.put(40.0, 960L); //open
        map.put(50.0, 1030L);
        map.put(60.0, 1080L);
        map.put(70.0, 1110L);
        map.put(80.0, 1170L);
        map.put(90.0, 1250L);
        map.put(100.0, 1280L);
        map.put(110.0, 1310L);
        map.put(115.0, 1330L);
        map.put(125.0, 1350L);
        map.put(130.0, 1400L);
        map.put(135.0, 1430L);
        map.put(140.0, 1460L);
        map.put(145.0, 1490L);
        map.put(150.0, 1550L);

        return new ShooterAimTask(new Pose(132, 136, 0), map);
    }
    public static ShooterAimTask defaultBlue() {// Deprecated
        TreeMap<Double, Long> map = new TreeMap<>();
        map.put(12.0, 850L);
        map.put(30.0, 960L); //open
        map.put(40.0, 960L); //open
        map.put(50.0, 1030L);
        map.put(60.0, 1080L);
        map.put(70.0, 1110L);
        map.put(80.0, 1170L);
        map.put(90.0, 1250L);
        map.put(100.0, 1280L);
        map.put(110.0, 1310L);
        map.put(115.0, 1330L);
        map.put(125.0, 1350L);
        map.put(130.0, 1400L);
        map.put(135.0, 1430L);
        map.put(140.0, 1460L);
        map.put(145.0, 1490L);
        map.put(150.0, 1550L);

        return new ShooterAimTask(new Pose(132, 136, 0).mirror(), map);
    }

    public double getDistanceToGoal(Pose currentPose) { // Math to get the distance to the goal.
        return Math.hypot(goalPose.getX() - currentPose.getX(), goalPose.getY() - currentPose.getY());
    }

    /**
     * Turn correction for auto-aim using heading PID (KP, KI, KD from MConstants).
     * Call resetHeadingPID() when driver releases aim to clear integral.
     */
    public double getTurnCorrection(Pose currentPose) { // Unused experimental code, WE DO NOT USE THIS CURRENTLY
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
