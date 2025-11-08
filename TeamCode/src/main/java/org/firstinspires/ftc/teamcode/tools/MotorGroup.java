package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MotorGroup {
    private DcMotor[] motors;     // For DcMotor types
    private DcMotorEx[] motorsEx; // For DcMotorEx types

    // Constructor for DcMotor group
    public MotorGroup(DcMotor... motors) {
        this.motors = motors;
        this.motorsEx = null; // No need to initialize DcMotorEx array
    }

    // Constructor for DcMotorEx group
    public MotorGroup(DcMotorEx... motorsEx) {
        this.motorsEx = motorsEx;
        this.motors = null; // No need to initialize DcMotor array
    }

    // Set power for regular motors (DcMotor)
    public void setPower(double power) {
        if (motors != null) {
            for (DcMotor motor : motors) {
                motor.setPower(power);
            }
        } else if (motorsEx != null) {
            for (DcMotorEx motorEx : motorsEx) {
                motorEx.setPower(power); // DcMotorEx also has setPower
            }
        }
    }

    // Set velocity for DcMotorEx motors
    public void setVelocity(long velocity, double power) {
        if (motorsEx != null) {
            for (DcMotorEx motorEx : motorsEx) {
                motorEx.setVelocity(velocity);
                motorEx.setPower(power); // Ensures motor is running
            }
        }
    }

    // Set mode for regular motors (DcMotor)
    public void setMode(DcMotor.RunMode mode) {
        if (motors != null) {
            for (DcMotor motor : motors) {
                motor.setMode(mode);
            }
        } else if (motorsEx != null) {
            for (DcMotorEx motorEx : motorsEx) {
                motorEx.setMode(mode);
            }
        }
    }

    // Set direction for regular motors (DcMotor)
    public void setDirection(DcMotor.Direction direction) {
        if (motors != null) {
            for (DcMotor motor : motors) {
                motor.setDirection(direction);
            }
        } else if (motorsEx != null) {
            for (DcMotorEx motorEx : motorsEx) {
                motorEx.setDirection(direction);
            }
        }
    }

    // Set zero power behavior for regular motors (DcMotor)
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        if (motors != null) {
            for (DcMotor motor : motors) {
                motor.setZeroPowerBehavior(behavior);
            }
        } else if (motorsEx != null) {
            for (DcMotorEx motorEx : motorsEx) {
                motorEx.setZeroPowerBehavior(behavior);
            }
        }
    }

    // Set encoder mode for regular motors (DcMotor)
    public void setUsingEncoder() {
        if (motors != null) {
            for (DcMotor motor : motors) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        } else if (motorsEx != null) {
            for (DcMotorEx motorEx : motorsEx) {
                motorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    // Get position range for regular motors (DcMotor)
    public int[] getPositionRange() {
        int max = Integer.MIN_VALUE;
        int min = Integer.MAX_VALUE;
        if (motors != null) {
            for (DcMotor motor : motors) {
                int pos = motor.getCurrentPosition();
                if (pos > max) max = pos;
                if (pos < min) min = pos;
            }
        } else if (motorsEx != null) {
            for (DcMotorEx motorEx : motorsEx) {
                int pos = motorEx.getCurrentPosition();
                if (pos > max) max = pos;
                if (pos < min) min = pos;
            }
        }
        return new int[]{min, max};
    }
}
