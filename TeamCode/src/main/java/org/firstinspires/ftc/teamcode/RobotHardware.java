package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {
    public static DcMotor frontLeftMotor;
    public static DcMotor backLeftMotor;
    public static DcMotor frontRightMotor;
    public static DcMotor backRightMotor;
    public static DcMotor intakeMotor;
    public static DcMotorEx outtakeMotor1;
    public static DcMotorEx outtakeMotor2;
    public static CRServo indexingWheel;

    public static void init(HardwareMap hwMap) {
        frontLeftMotor = hwMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hwMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hwMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hwMap.get(DcMotor.class, "backRightMotor");
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        outtakeMotor1 = hwMap.get(DcMotorEx.class, "outtakeMotor1");
        outtakeMotor2 = hwMap.get(DcMotorEx.class, "outtakeMotor2");
        indexingWheel = hwMap.get(CRServo.class, "indexingWheel");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
