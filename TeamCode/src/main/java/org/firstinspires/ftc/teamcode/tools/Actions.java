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

    private final Timer actionTimer = new Timer();

    private boolean isShooting = false;
    private boolean shooterSpinningUp = false;

    // ========== Intake ==========
    public void intakeOn(double power) {
        intakeMotor.setPower(power);
    }

    public void intakeOff() {
        intakeMotor.setPower(0);
    }

    // ========== Outtake / Shooter ==========
    /**
     * Starts shooter spin-up sequence.
     * Call this before shoot() or use shootAuto() for automatic timing.
     */
    public void spinUpShooter(long velocity, double power) {
        outtakeMotors.setVelocity(velocity, power);
        shooterSpinningUp = true;
        actionTimer.resetTimer();
    }


    public void stopShooter() {
        outtakeMotors.setVelocity(0, 0);
        shooterSpinningUp = false;
        isShooting = false;
    }

    /**
     * Shoots one ball. Make sure shooter is spun up
     */
    public void shoot() {
        indexingWheel.setPower(1);
        isShooting = true;
        actionTimer.resetTimer();
    }

    public void stopIndex() {
        indexingWheel.setPower(0);
        isShooting = false;
    }

    /**
     * Combined automatic shoot sequence:
     * Spins up >> waits >> indexes >> stops.
     * Returns true when finished.
     */
    public boolean shootAuto(long velocity, double power, long spinUpMs, long indexMs, long elapsedTime) {
        long elapsed = elapsedTime; //|| actionTimer.getElapsedTime();

        if (!shooterSpinningUp) {
            spinUpShooter(velocity, power);
        } else if (elapsed >= spinUpMs && !isShooting) {
            shoot();
        } else if (elapsed >= spinUpMs + indexMs) {
            stopIndex();
            stopShooter();
            return true; // done
        }

        return false; // still running
    }
}
