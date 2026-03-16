package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.hardware.AdafruitNeoPixel;

public class RobotHardware {
    public static DcMotorEx frontLeftMotor;
    public static DcMotorEx backLeftMotor;
    public static DcMotorEx frontRightMotor;
    public static DcMotorEx backRightMotor;
    public static DcMotorEx intakeMotor;
    public static DcMotorEx outtakeMotor;
    public static MotorEx outtakeMotorSolver;
    public static Servo indexerPaddle;
    public static CRServo indexerWheel0;
    public static CRServo indexerWheel1;
    public static Servo outtakeAngleAdjust;
    public static DcMotorEx turret;

    public static Servo blocker;

    /** Ball detection: 0= intakeLeft, 1= intakeRight, 2= indexer, 3= launch ready (Rev Color V3 as DistanceSensor). */
    public static DistanceSensor ballSensor0;
    public static DistanceSensor ballSensor1;
    public static DistanceSensor ballSensor2;
    public static DistanceSensor ballSensor3;

    public static AdafruitNeoPixel pixel;

    public static void init(HardwareMap hwMap) {
        frontLeftMotor = hwMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeftMotor = hwMap.get(DcMotorEx.class, "backLeftMotor");
        frontRightMotor = hwMap.get(DcMotorEx.class, "frontRightMotor");
        backRightMotor = hwMap.get(DcMotorEx.class, "backRightMotor");
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");

        outtakeMotor = hwMap.get(DcMotorEx.class, "outtakeMotor");
//        outtakeMotorSolver = new MotorEx(hwMap, "outtakeMotor");
//        outtakeMotorSolver.setInverted(true);

        indexerPaddle = hwMap.get(Servo.class, "indexerPaddle");
        indexerWheel0 = hwMap.get(CRServo.class, "indexerWheel0");
        indexerWheel1 = hwMap.get(CRServo.class, "indexerWheel1");
        outtakeAngleAdjust = hwMap.get(Servo.class, "outtakeAngleAdjust");

        blocker = hwMap.get(Servo.class, "blocker");

        ballSensor0 = hwMap.get(DistanceSensor.class, "ball3L");
        ballSensor1 = hwMap.get(DistanceSensor.class, "ball3R");
        ballSensor2 = hwMap.get(DistanceSensor.class, "ball2");
        ballSensor3 = hwMap.get(DistanceSensor.class, "ball1");

        turret = hwMap.get(DcMotorEx.class, "turretMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        pixel = hwMap.get(AdafruitNeoPixel.class, "pixel");
    }
}
