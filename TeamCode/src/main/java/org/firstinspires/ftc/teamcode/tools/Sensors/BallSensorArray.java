package org.firstinspires.ftc.teamcode.tools.Sensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.MConstants;

import java.util.EnumMap;

/**
 * Wraps 4 color/distance sensors to detect ball presence at different conveyor positions.
 * Uses proximity: ball present = distance below threshold (e.g. Rev Color Sensor V3 distance).
 */
public class BallSensorArray {
    private static class BallSensor {
        final DistanceSensor sensor;
        final double thresholdCm;

        BallSensor(DistanceSensor sensor, double thresholdCm) {
            this.sensor = sensor;
            this.thresholdCm = thresholdCm;
        }

        boolean hasBall() {
            if (sensor == null) return false;
            try {
                double cm = sensor.getDistance(DistanceUnit.CM);
                return cm > 0 && cm < thresholdCm;
            } catch (Exception e) {
                return false;
            }
        }

        double distanceCm() {
            if (sensor == null) return Double.NaN;
            try {
                return sensor.getDistance(DistanceUnit.CM);
            } catch (Exception e) {
                return Double.NaN;
            }
        }
    }

    private final EnumMap<BallPosition, BallSensor> sensors =
            new EnumMap<>(BallPosition.class);

    public BallSensorArray() {
        sensors.put(BallPosition.FUNNEL_LEFT,
                new BallSensor(RobotHardware.ballSensor0, MConstants.BALL_DETECT_DISTANCE_CM[0]));
        sensors.put(BallPosition.FUNNEL_RIGHT,
                new BallSensor(RobotHardware.ballSensor1, MConstants.BALL_DETECT_DISTANCE_CM[1]));
        sensors.put(BallPosition.TUNNEL,
                new BallSensor(RobotHardware.ballSensor2, MConstants.BALL_DETECT_DISTANCE_CM[2]));
        sensors.put(BallPosition.LAUNCH_READY,
                new BallSensor(RobotHardware.ballSensor3, MConstants.BALL_DETECT_DISTANCE_CM[3]));
    }

    public boolean hasBallAt(BallPosition position) {
        return sensors.get(position).hasBall();
    }

    public double getDistanceCm(BallPosition position) {
        return sensors.get(position).distanceCm();
    }

    public int getBallCount() {
        int count = 0;
        for (BallPosition p : BallPosition.values()) {
            if (hasBallAt(p)) count++;
        }
        return count;
    }

    public boolean isLaunchReady() {
        return hasBallAt(BallPosition.LAUNCH_READY);
    }
}