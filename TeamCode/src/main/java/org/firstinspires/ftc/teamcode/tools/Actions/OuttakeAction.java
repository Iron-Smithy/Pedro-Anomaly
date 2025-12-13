// OuttakeAction.java
package org.firstinspires.ftc.teamcode.tools.Actions;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class OuttakeAction {
    private final DcMotorEx motor;
    private double targetVelocity = 0;
    private long stableSince = -1;
    private final long requiredStableTimeMs = 100;

    private final double tolerance = 30; // adjustable TPS tolerance

    public OuttakeAction(HardwareMap hardwareMap) {
        motor = RobotHardware.outtakeMotor;
        motor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void spinUp(double tps) {
        targetVelocity = tps;
        stableSince = -1;
        motor.setVelocity(tps);
    }


    public boolean stop() {
        targetVelocity = 0;
        motor.setVelocity(0);
        return true;
    }

//    public boolean isAtTargetVelocity() {
//        return Math.abs(motor.getVelocity() - targetVelocity) <= tolerance;
//    }
    public boolean isAtTargetVelocity() {
        if (targetVelocity <= 0) return false;
        return Math.abs(motor.getVelocity() - targetVelocity) <= tolerance;
    }
//    public boolean isAtTargetVelocity() {
//        if (targetVelocity <= 0) {
//            stableSince = -1;
//            return false;
//        }
//
//        double error = Math.abs(motor.getVelocity() - targetVelocity);
//
//        if (error <= tolerance) {
//            if (stableSince < 0) {
//                stableSince = System.currentTimeMillis();
//            }
//            return System.currentTimeMillis() - stableSince >= requiredStableTimeMs;
//        } else {
//            stableSince = -1;
//            return false;
//        }
//    }
}
