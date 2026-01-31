// OuttakeAction.java
package org.firstinspires.ftc.teamcode.tools.Actions;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.MConstants;

/**
 * Outtake (shooter) motor with velocity PID: power = KV*targetVel + P*(targetVel - actualVel) + KS.
 * Reduces oscillation and improves speed stability. Call update() every loop.
 */
public class OuttakeAction {
    public final DcMotorEx motor;
    private double targetVelocity = 0;
    private final double tolerance = 30; // TPS tolerance for "at speed"

    public OuttakeAction(HardwareMap hardwareMap) {
        motor = RobotHardware.outtakeMotor;
        motor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void spinUp(double tps) {
        targetVelocity = tps;
    }

    /**
     * Call every loop. Applies KV+P+KS to motor power for stable speed.
     */
    public void update() {
        if (targetVelocity <= 0) {
            motor.setPower(0);
            return;
        }
        double power = getShooterPower();
        power = Math.max(0, Math.min(1, power));
        motor.setPower(power);
    }

    private double getShooterPower() {
        double velErr = targetVelocity - motor.getVelocity();
        return (MConstants.SHOOTER_KV * targetVelocity)
                + (MConstants.SHOOTER_P * velErr)
                + MConstants.SHOOTER_KS;
    }

    public boolean stop() {
        targetVelocity = 0;
        motor.setPower(0);
        return true;
    }

    public boolean isAtTargetVelocity() {
        if (targetVelocity <= 0) return false;
        return Math.abs(motor.getVelocity() - targetVelocity) <= tolerance;
    }
}
