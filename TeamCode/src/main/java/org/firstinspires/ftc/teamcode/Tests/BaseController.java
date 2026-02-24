//package org.firstinspires.ftc.teamcode.Tests;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.Actions.TurretAction;
//import org.firstinspires.ftc.teamcode.RobotHardware;
//
//@Configurable
//@TeleOp (name = "Turret Controller", group = "LinearOpMode")
//public class BaseController extends OpMode {
//
//    private TurretAction turret;
//
//    @Override
//    public void init() {
//        RobotHardware.init(hardwareMap);
//
//        turret = new TurretAction(hardwareMap);
//        turret.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // reset to 0);
//    }
//
//    @Override
//    public void start() {
//
//    }
//
//    @Override
//    public void loop() {
//        double joystickAngle = Math.atan2(gamepad1.right_stick_x, -gamepad1.right_stick_y);
//
//        double wrappedAngle = turret.calculateWrapAngles(joystickAngle);
//        double limitedAngle = turret.limitRadValues(wrappedAngle);
//
//        int tickRaw = turret.radToTick(limitedAngle);
//
//        turret.runToTick(tickRaw);
//
//        telemetry.addData("joystick angle", joystickAngle);
//        telemetry.addData("joystick angle wrapped", wrappedAngle);
//        telemetry.addData("wrapped angle limited", limitedAngle);
//
//        telemetry.addData("tick target raw", tickRaw);
//
//        telemetry.addData("turret target Position", turret.motor.getTargetPosition());
//        telemetry.addData("real turret position", turret.motor.getCurrentPosition());
//
//        telemetry.update();
//    }
//}