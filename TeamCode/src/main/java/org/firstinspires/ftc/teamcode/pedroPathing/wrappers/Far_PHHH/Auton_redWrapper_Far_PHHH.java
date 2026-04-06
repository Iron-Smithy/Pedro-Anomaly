package org.firstinspires.ftc.teamcode.pedroPathing.wrappers.Far_PHHH;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Auton.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Auton.AutonB_Far_PHHH_0;

@Autonomous(name = "Auton Red H H H", group = "Far")
public class Auton_redWrapper_Far_PHHH extends AutonB_Far_PHHH_0 {
    @Override
    public void init() {
        setAlliance(Alliance.RED);
        super.init();
    }
}
