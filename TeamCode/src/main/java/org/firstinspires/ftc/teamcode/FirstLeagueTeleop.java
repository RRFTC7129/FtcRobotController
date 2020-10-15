package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp(name="First League Teleop", group="OpMode")
public class FirstLeagueTeleop extends OpMode {

    FirstLeagueBase base;

    @Override
    public void init() {
        base = new FirstLeagueBase(this);
        base.retrieveHeading();
    }

    @Override
    public void loop() {
        base.postTelemetry();
        base.resetHeading(); //Allows for manually resetting the robot heading in teleop to correct the field centric code
        base.updateDriveTrain(); //Field Centric Mecanum Drive Code
    }
}