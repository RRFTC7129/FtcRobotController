package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//@Disabled
@Autonomous(name="Celebrimbor Auto", group="")
public class CelebrimborAuto extends LinearOpMode {

    CelebrimborBase base;

    @Override
    public void runOpMode() throws InterruptedException {
        base = new CelebrimborBase(this);
        base.selection();
        waitForStart();
        base.timerOpMode.reset();
        base.driveSomewhere();
        //base.waitForEnd();
        //base.storeHeading();
    }
}