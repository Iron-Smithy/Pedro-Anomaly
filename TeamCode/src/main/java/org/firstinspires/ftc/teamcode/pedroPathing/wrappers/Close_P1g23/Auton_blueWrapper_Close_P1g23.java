package org.firstinspires.ftc.teamcode.pedroPathing.wrappers.Close_P1g23;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Auton.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Auton.AutonB_Close_P1g23_0;

@Autonomous(name = "Auton Blue 1 g 2 3", group = "Close")
public class Auton_blueWrapper_Close_P1g23 extends AutonB_Close_P1g23_0 {
    @Override
    public void init() {
        setAlliance(Alliance.BLUE);
        super.init();
    }
}
