package org.firstinspires.ftc.teamcode.pedroPathing.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.tools.Actions.EjectorAction;
import org.firstinspires.ftc.teamcode.tools.Actions.IndexAction;
import org.firstinspires.ftc.teamcode.tools.Actions.IntakeAction;
import org.firstinspires.ftc.teamcode.tools.Actions.OuttakeAction;
import org.firstinspires.ftc.teamcode.pedroPathing.MConstants;
import org.firstinspires.ftc.teamcode.tools.ButtonHandler;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class PDriveAR extends OpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(/*144-*/120, 127, Math.toRadians(/*180-*/37));
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.25;

    double fieldCentricOffset = 0;


    double k = 2.0;
    double expKMinus1 = Math.exp(k) - 1; // Precompute at init;


    private ButtonHandler outtakeControl = new ButtonHandler();
    private ButtonHandler outtakeVelocityU = new ButtonHandler();
    private ButtonHandler outtakeVelocityD = new ButtonHandler();
    boolean outtakeON = false;
    long outtakeSpeed = 1000;
    long outtakeSpeedMin = 500;
    long outtakeSpeedMax = 3000;

    private ButtonHandler intakeDirectionControlU = new ButtonHandler();
    private ButtonHandler intakeDirectionControlD = new ButtonHandler();
    double intakePow = 0;


    private IntakeAction intake;
    private IndexAction indexer;
    private EjectorAction ejector;
    private OuttakeAction outtake;

    @Override
    public void init() {
        RobotHardware.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(/*144-*/100, 107))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(/*180-*/47), 0.8))
                .build();

        intake = new IntakeAction(hardwareMap);
        indexer = new IndexAction(hardwareMap);
        ejector = new EjectorAction(hardwareMap);
        outtake = new OuttakeAction(hardwareMap);

        outtakeControl.setOnPress(() -> {
            outtakeON = !outtakeON;
        });
        outtakeVelocityU.setOnPress(() -> {
            outtakeSpeed = Math.min(outtakeSpeedMax, Math.max(outtakeSpeedMin, outtakeSpeed + 150));
        });
        outtakeVelocityD.setOnPress(() -> {
            outtakeSpeed = Math.min(outtakeSpeedMax, Math.max(outtakeSpeedMin, outtakeSpeed - 150));
        });

        intakeDirectionControlD.setOnPress(() -> intakePow = (intakePow > 0 ? 0 : 1));
        intakeDirectionControlU.setOnPress(() -> intakePow = (intakePow < 0 ? 0 : -1));
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive(true);
        outtake.spinUp(outtakeSpeed);
        outtake.stop();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

//        follower.setPose(aprilPose);

        outtakeControl.update(gamepad1.left_trigger, 0.25f);
        outtakeVelocityU.update(gamepad1.dpad_up);
        outtakeVelocityD.update(gamepad1.dpad_down);

        intakeDirectionControlU.update(gamepad1.right_bumper);
        intakeDirectionControlD.update(gamepad1.circle);

        if (!automatedDrive) {
            double heading = follower.getHeading() - fieldCentricOffset;

            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x / 2;

            // Exponential shaping
            double driveY = Math.signum(y) * (Math.exp(k * Math.abs(y)) - 1) / expKMinus1;
            double driveX = Math.signum(x) * (Math.exp(k * Math.abs(x)) - 1) / expKMinus1;

            // Slow mode scaling
            if (slowMode) {
                driveX *= slowModeMultiplier;
                driveY *= slowModeMultiplier;
                turn   *= slowModeMultiplier;
            }

            // *** Field-centric Pedro control ***
//            follower.driveFieldCentric(
//                    driveX,
//                    driveY,
//                    turn,
//                    heading
//            );
        }


        if (gamepad1.shareWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }
        if (automatedDrive && (gamepad1.psWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }
        if (gamepad1.optionsWasPressed()) {
            fieldCentricOffset = follower.getHeading(); 
        }

        if (outtakeON) {
            outtake.spinUp(outtakeSpeed);
            long diff = outtakeSpeedMax-outtakeSpeedMin;
            double percent = outtake.isAtTargetVelocity() ? 0.8 : 0.1;

            gamepad1.rumble(percent, percent, 50);
        } else {
            outtake.stop();
            gamepad1.rumble(0, 0, 0);
        }
        if (Math.abs(intakePow) > 0) {
            intake.spin(intakePow);
        } else {
            intake.stop();
        }
        if (gamepad1.left_bumper) {
            indexer.runIn();
        } else {
            indexer.stop();
        }
        if (gamepad1.a) {
            ejector.up();
        } else {
            ejector.down();
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

        telemetry.addData("slowModeMultiplier", slowModeMultiplier);
        telemetry.addData("slow on?", slowMode);
        telemetry.addData("outtake On?", outtakeON);
        telemetry.addData("outtake speed", outtakeSpeed);
        telemetry.update();
    }
}