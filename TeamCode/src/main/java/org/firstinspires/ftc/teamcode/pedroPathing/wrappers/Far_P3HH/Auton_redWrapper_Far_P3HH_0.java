package org.firstinspires.ftc.teamcode.pedroPathing.wrappers.Far_P3HH;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Auton.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Auton.AutonB_Far_P3HH_0;

@Autonomous(name = "Auton Red Far Double Human", group = "Far")
public class Auton_redWrapper_Far_P3HH_0 extends AutonB_Far_P3HH_0 {
    @Override
    public void init() {
        setAlliance(Alliance.RED);
        super.init();
    }
}
