package org.firstinspires.ftc.teamcode.teleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Auton.Alliance;

@Configurable
@TeleOp (name = "teleOp RED", group = "AAA")
public class teleOp_redWrapper extends teleOp_B_0 {
    @Override
    public void init() {
        setAlliance(Alliance.RED);
        super.init();
    }
}
