package org.firstinspires.ftc.teamcode.pedroPathing.wrappers.Far_P23GH;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Auton.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Auton.AutonB_Far_P23GH_0;

@Autonomous(name = "Auton Red Far Gate", group = "Far")
public class Auton_redWrapper_Far_P23GH extends AutonB_Far_P23GH_0 {
    @Override
    public void init() {
        setAlliance(Alliance.RED);
        super.init();
    }
}
