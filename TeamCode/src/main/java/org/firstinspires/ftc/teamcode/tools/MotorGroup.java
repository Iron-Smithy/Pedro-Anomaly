package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorGroup {
    private DcMotor[] motors;

    public MotorGroup(DcMotor... motors) { this.motors = motors; }

    public void setPower(double power) {
        for (DcMotor motor : motors) motor.setPower(power);
    }

    public void setMode(DcMotor.RunMode mode) {
        for (DcMotor motor : motors) motor.setMode(mode);
    }

    public void setDirection(DcMotor.Direction direction) {
        for (DcMotor motor : motors) motor.setDirection(direction);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        for (DcMotor motor : motors) motor.setZeroPowerBehavior(behavior);
    }

    public void setUsingEncoder() {
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public int[] getPositionRange() {
        int max = Integer.MIN_VALUE;
        int min = Integer.MAX_VALUE;
        for (DcMotor motor : motors) {
            int pos = motor.getCurrentPosition();
            if (pos > max) max = pos;
            if (pos < min) min = pos;
        }
        return new int[]{min, max};
    }
}