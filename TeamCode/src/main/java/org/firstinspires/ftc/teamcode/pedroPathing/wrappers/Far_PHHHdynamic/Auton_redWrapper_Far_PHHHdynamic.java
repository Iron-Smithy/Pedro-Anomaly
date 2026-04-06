package org.firstinspires.ftc.teamcode.pedroPathing.wrappers.Far_PHHHdynamic;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Auton.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Auton.AutonB_Far_PHHHdynamic_0;

@Autonomous(name = "Auton Red H H H dynamic", group = "Far")
public class Auton_redWrapper_Far_PHHHdynamic extends AutonB_Far_PHHHdynamic_0 {
    @Override
    public void init() {
        setAlliance(Alliance.RED);
        super.init();
    }
}
