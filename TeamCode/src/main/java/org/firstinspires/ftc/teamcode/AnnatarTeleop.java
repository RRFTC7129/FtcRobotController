package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//@Disabled
@TeleOp(name="Annatar Teleop", group="OpMode")
public class AnnatarTeleop extends OpMode {

    AnnatarBase base;

    @Override
    public void init() {
        base = new AnnatarBase(this);
        base.retrieveHeading(); //Pull the final heading from autonomous and use it to correct the field centric code
        base.resetEncoders();
        base.runWithoutEncoders();
    }

    @Override
    public void loop() {
        base.postTelemetry();
        base.resetHeading(); //Allows for manually resetting the robot heading in teleop to correct the field centric code
        base.updateDriveTrain(); //Field Centric Mecanum Drive Code
        base.controlCollection(); //Collection and Transfer Controls
        base.flywheelSpeed(); //Logic for Varying Launcher Speed
        base.controlLauncher(); //Launcher Controls
        base.controlTransfer(); //Transfer Controls
        base.autoTransfer(); //Automatically lift rings once in transfer
        base.launchAngle(); //Change Launch Angle
        base.controlWobbleGoal(); //Wobble Goal Controls
        base.autoPowerShots();
    }
}