package org.firstinspires.ftc.teamcode.tools;

import static org.firstinspires.ftc.teamcode.RobotHardware.*;
import org.firstinspires.ftc.teamcode.tools.MotorGroup;

import com.pedropathing.util.Timer;

/**
 * Handles combined mechanisms,
 * so autonomous can be easy.
 */
public class Actions {
    MotorGroup outtakeMotors = new MotorGroup(outtakeMotor1, outtakeMotor2);

    // ========== Intake ==========
    public void intakeOn(double power) {
        intakeMotor.setPower(power);
    }
    public void intakeOff() {
        intakeMotor.setPower(0);
    }

    // ========== Indexer ==========

    public void indexOn(double power) {
        indexingWheel.setPower(power);
    }
    public void indexOff() {
        indexingWheel.setPower(0);
    }

    // ========== Outtake / Shooter ==========
    public void outtakeSpinUp(long TPS, double power) {
        outtakeMotors.setVelocity(TPS, power);
    }
    public void stopShooter() {
        outtakeMotors.setVelocity(0, 0);
    }
    public void spinUpShooter(long velocity, double power) {
        outtakeMotors.setVelocity(velocity, power);
    }

    // ========== Functions ==========
}
