//// Field Centric Mecanum TeleOp
//package org.firstinspires.ftc.teamcode.teleop;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//
//import org.firstinspires.ftc.teamcode.RobotHardware;
//import org.firstinspires.ftc.teamcode.tools.ButtonHandler;
//import org.firstinspires.ftc.teamcode.tools.MotorGroup;
//import static org.firstinspires.ftc.teamcode.RobotHardware.*;
//
//@TeleOp
//public class FCMT extends LinearOpMode {
//    private boolean intakeToggle = false;
//    private int intakeDir = -1;
//    private int outtakeDir = 1; // 1 outtake
//    private double outtakePow = 0.5;
//    private double indexingWheelDir = 0; // 1 intake
//
//    double k = 2.0;
//    double expKMinus1 = Math.exp(k) - 1; // Precompute at init;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        RobotHardware.init(hardwareMap);
//
//        MotorGroup outtakeMotors = new MotorGroup(outtakeMotor1, outtakeMotor2);
//        outtakeMotors.setUsingEncoder();
//
//        MotorGroup driveMotors = new MotorGroup(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
//
//        ButtonHandler leftPaddle = new ButtonHandler();
//        ButtonHandler rightPaddle = new ButtonHandler();
//        ButtonHandler leftTrigger = new ButtonHandler();
//        ButtonHandler rightTrigger = new ButtonHandler();
//        ButtonHandler DPadUP = new ButtonHandler();
//        ButtonHandler DPadDown = new ButtonHandler();
//
//        // Intake
//        rightPaddle.setOnPress(() -> intakeDir = (intakeDir == 1 ? 0 : 1));
//        rightTrigger.setOnPress(() -> intakeDir = (intakeDir == -1 ? 0 : -1));
//
//        // Outtake
//        leftPaddle.setOnHold(() -> indexingWheelDir = -1);
//        leftPaddle.setOnRelease(() -> indexingWheelDir = 0);
//        leftTrigger.setOnPress(() -> outtakeDir = 1 - outtakeDir);
//
//        DPadUP.setOnPress(() -> outtakePow = Math.min(1.0, Math.max(0, outtakePow + 0.1)));
//        DPadDown.setOnPress(() -> outtakePow = Math.min(1.0, Math.max(0, outtakePow - 0.1)));
//
//        driveMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()) {
//            leftPaddle.update(gamepad1.left_bumper);
//            rightPaddle.update(gamepad1.right_bumper);
//
//            leftTrigger.update(gamepad1.left_trigger, 0.25f);
//            rightTrigger.update(gamepad1.right_trigger, 0.25f);
//
//            DPadUP.update(gamepad1.dpad_up);
//            DPadDown.update(gamepad1.dpad_down);
//
//            // 1. Read joystick
//            double jy = -gamepad1.left_stick_y;
//            double jx = gamepad1.left_stick_x;
//            double jrx = gamepad1.right_stick_x;
//
//            // 2. Exponential stick curve
//            double y = Math.signum(jy) * (Math.exp(k * Math.abs(jy)) - 1) / expKMinus1;
//            double x = Math.signum(jx) * (Math.exp(k * Math.abs(jx)) - 1) / expKMinus1;
//
//            double botHeading = 0; //RobotHardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//            // 5. Field-centric drive transform
//            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//            rotX *= 1.1;  // Strafing compensation
//
//            // 6. Motor powers
//            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(jrx), 1);
//            frontLeftMotor.setPower((rotY + rotX + jrx) / denominator);
//            backLeftMotor.setPower((rotY - rotX + jrx) / denominator);
//            frontRightMotor.setPower((rotY - rotX - jrx) / denominator);
//            backRightMotor.setPower((rotY + rotX - jrx) / denominator);
//
//            intakeMotor.setPower(intakeDir);
//            outtakeMotors.setPower(outtakePow * outtakeDir);
//            indexingWheel.setPower(gamepad1.a ? indexingWheelDir*-1 : indexingWheelDir);
//
//            telemetry.addData("Outtake Speed", outtakePow);
//            telemetry.addData("Outtake Dir", outtakeDir);
//            telemetry.addData("Motor Speed", outtakePow * outtakeDir);
//            telemetry.update();
//        }
//    }
//}