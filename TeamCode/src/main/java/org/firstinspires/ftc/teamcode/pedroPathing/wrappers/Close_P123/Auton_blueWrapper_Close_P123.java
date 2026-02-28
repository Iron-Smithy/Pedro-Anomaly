package org.firstinspires.ftc.teamcode.pedroPathing.wrappers.Close_P123;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Auton.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Auton.AutonB_Close_P123_0;

@Autonomous(name = "Auton Blue 12P", group = "Close")
public class Auton_blueWrapper_Close_P123 extends AutonB_Close_P123_0 {
    @Override
    public void init() {
        setAlliance(Alliance.BLUE);
        super.init();
    }
}
