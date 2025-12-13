// OuttakeAction.java
package org.firstinspires.ftc.teamcode.tools.Actions;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.MConstants;

public class OuttakeAction {
    private final DcMotorEx motor;
    private final MotorEx motorSolver;
    private final PIDFController controller = new PIDFController(MConstants.shooterCoeff);
    private double targetVelocity = 0;
    private long stableSince = -1;
    private final long requiredStableTimeMs = 100;

    private final double tolerance = 30; // adjustable TPS tolerance

    public OuttakeAction(HardwareMap hardwareMap) {
        motor = RobotHardware.outtakeMotor;
        motor.setDirection(DcMotorEx.Direction.REVERSE);

        motorSolver = RobotHardware.outtakeMotorSolver;
    }

    public void spinUp(double tps) {
        targetVelocity = tps;
        stableSince = -1;
//        controller.setSetPoint(tps);
//        if (tps == 0) {
//            motorSolver.stopMotor();
//        }
//        motorSolver.set(controller.calculate(motorSolver.getCorrectedVelocity()));
        motor.setVelocity(tps);
    }

    public void update() {}

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
}