package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Actions.TurretAction;

@Configurable
@TeleOp (name = "Turret Controller", group = "LinearOpMode")
public class TurretController extends OpMode {

    private TurretAction turret;

    public static double TURRET_p = 0; // 0;
    public static double TURRET_i = 0; // 0;
    public static double TURRET_d = 0; // 0;
    public static double TURRET_f = 0; // 0;

    @Override
    public void init() {
        RobotHardware.init(hardwareMap);

        turret = new TurretAction(hardwareMap);
        turret.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // reset to 0);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if (gamepad1.triangleWasPressed()) {
            turret.updatePIDF(new com.pedropathing.control.PIDFCoefficients(TURRET_p, TURRET_i, TURRET_d, TURRET_f));
            telemetry.addLine("Updated");
        }

        double joystickAngle = Math.atan2(gamepad1.right_stick_x, -gamepad1.right_stick_y);

        double wrappedAngle = turret.calculateWrapAngles(joystickAngle);
        double limitedAngle = turret.limitRadValues(wrappedAngle);

        int tickRaw = turret.radToTick(limitedAngle);

        turret.runToTick(tickRaw);

        telemetry.addData("joystick angle", joystickAngle);
        telemetry.addData("joystick angle wrapped", wrappedAngle);
        telemetry.addData("wrapped angle limited", limitedAngle);

        telemetry.addData("tick target raw", tickRaw);

        telemetry.addData("turret target Position", turret.controller.getTargetPosition());
        telemetry.addData("real turret position", turret.motor.getCurrentPosition());

        telemetry.update();
    }
}