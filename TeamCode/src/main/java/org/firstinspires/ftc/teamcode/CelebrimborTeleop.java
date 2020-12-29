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
        base.retrieveHeading(); //Pull the final heading from autonomous and use it to correct the field centric code
    }

    @Override
    public void loop() {
        base.postTelemetry();
        base.resetHeading(); //Allows for manually resetting the robot heading in teleop to correct the field centric code
        base.updateDriveTrain(); //Field Centric Mecanum Drive Code
        base.controlCollection(); //Collection and Transfer Controls
        base.dpadStuffs(); //Logic for Varying Launcher Speed
        base.controlLauncher(); //Launcher Controls
        base.controlWobbleGoal(); //Wobble Goal Controls
        base.unlatchCollection(); //In case the collection doesn't come down in autonomous, the driver can fix that here
    }
}