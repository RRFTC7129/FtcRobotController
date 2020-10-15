package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Disabled
@Autonomous(name="First League Auto", group="")
public class FirstLeagueAuto extends LinearOpMode {

    FirstLeagueBase base;

    @Override
    public void runOpMode() throws InterruptedException {
        base = new FirstLeagueBase(this);
        base.selection();
        waitForStart();
        base.timerOpMode.reset();
        base.waitForEnd();
        base.storeHeading();
    }
}
