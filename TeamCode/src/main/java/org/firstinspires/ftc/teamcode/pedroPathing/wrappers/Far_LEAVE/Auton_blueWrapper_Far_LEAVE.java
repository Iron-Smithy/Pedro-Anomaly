package org.firstinspires.ftc.teamcode.pedroPathing.wrappers.Far_LEAVE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Auton.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Auton.AutonB_Far_LEAVE_0;

@Autonomous(name = "Auton Blue LEAVE", group = "Far")
public class Auton_blueWrapper_Far_LEAVE extends AutonB_Far_LEAVE_0 {
    @Override
    public void init() {
        setAlliance(Alliance.BLUE);
        super.init();
    }
}
