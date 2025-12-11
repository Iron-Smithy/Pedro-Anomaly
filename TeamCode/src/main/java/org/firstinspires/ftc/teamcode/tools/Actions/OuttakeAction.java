// OuttakeAction.java
package org.firstinspires.ftc.teamcode.tools.Actions;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class OuttakeAction {
    private final DcMotorEx motor;
    private double targetVelocity = 0;

    private final double tolerance = 50; // adjustable TPS tolerance

    public OuttakeAction(HardwareMap hardwareMap) {
        motor = RobotHardware.outtakeMotor;
        motor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void spinUp(double tps) {
        targetVelocity = tps;
        motor.setVelocity(tps);
    }

    public boolean stop() {
        targetVelocity = 0;
        motor.setVelocity(0);
        return true;
    }

    public boolean isAtTargetVelocity() {
        return Math.abs(motor.getVelocity() - targetVelocity) <= tolerance;
    }
}
