package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class RobotHardware {
    public static DcMotor frontLeftMotor;
    public static DcMotor backLeftMotor;
    public static DcMotor frontRightMotor;
    public static DcMotor backRightMotor;
    public static DcMotor intakeMotor;
    public static DcMotorEx outtakeMotor;
    public static MotorEx outtakeMotorSolver;
    public static Servo indexerPaddle;
    public static CRServo indexerWheel0;
    public static CRServo indexerWheel1;
    public static Servo outtakeAngleAdjust;

    public static void init(HardwareMap hwMap) {
        frontLeftMotor = hwMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hwMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hwMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hwMap.get(DcMotor.class, "backRightMotor");
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");

        outtakeMotor = hwMap.get(DcMotorEx.class, "outtakeMotor");
        outtakeMotorSolver = new MotorEx(hwMap, "outtakeMotor");
        outtakeMotorSolver.setInverted(true);

        indexerPaddle = hwMap.get(Servo.class, "indexerPaddle");
        indexerWheel0 = hwMap.get(CRServo.class, "indexerWheel0");
        indexerWheel1 = hwMap.get(CRServo.class, "indexerWheel1");
        outtakeAngleAdjust = hwMap.get(Servo.class, "outtakeAngleAdjust");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
