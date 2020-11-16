package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//@Disabled
@TeleOp(name="Celebrimbor Teleop", group="OpMode")
public class CelebrimborTeleop extends OpMode {

    CelebrimborBase base;

    @Override
    public void init() {
        base = new CelebrimborBase(this);
        base.retrieveHeading();
    }

    @Override
    public void loop() {
        base.postTelemetry();
        base.resetHeading(); //Allows for manually resetting the robot heading in teleop to correct the field centric code
        base.updateDriveTrain(); //Field Centric Mecanum Drive Code
        base.controlCollection();
        base.dpadStuffs();
        base.controlLauncher();
        base.controlWobbleGoal();
    }
}