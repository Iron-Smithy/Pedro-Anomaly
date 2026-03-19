package org.firstinspires.ftc.teamcode.pedroPathing.wrappers.Far_P23GH;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Auton.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Auton.AutonB_Far_P23GH_0;

@Autonomous(name = "Auton Blue Far Gate", group = "Far")
public class Auton_blueWrapper_Far_P23GH extends AutonB_Far_P23GH_0 {
    @Override
    public void init() {
        setAlliance(Alliance.BLUE);
        super.init();
    }
}
