package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.openftc.easyopencv.OpenCvCamera;

import java.io.File;

class CelebrimborBase {
    OpMode opMode;

    //Declare all devices
    DcMotor motorDriveLF, motorDriveLB, motorDriveRF, motorDriveRB, motorArm, motorCollection, motorLaunchL, motorLaunchR;
    CRServo servoGoal, servoCollectionRaise;
    Servo servoLimit;
    BNO055IMU imu;                   // IMU Gyro itself
    Orientation angles;              // IMU Gyro's Orienting
    //Camera
    OpenCvCamera webcam;
    // Stores the robot's heading at the end of autonomous to be used in teleop
    File headingFile = AppUtil.getInstance().getSettingsFile("headingFile");
    // Timers
    ElapsedTime timerOpMode;         // Tells us how long the Opmode is running since it started.
    ElapsedTime timerTravel;         // Used to cancel a robot turn in case we lock up.
    ElapsedTime timerLauncher;       // Used to stop the firing function if it's taking too long
    ElapsedTime timerSpeedControl;   // Used in the proportional speed control for the launcher

    // Variables used in Path Selection
    String allianceColor;                                     // What's our alliance color?
    String parkingPreference;                                 // Where do we want to park at the end?
    // Variables used in Odometry
    double leftWheelTickDelta = 0;                            // Change in left encoder rotation
    double rightWheelTickDelta = 0;                           // Change in right encoder rotation
    double strafeWheelTickDelta = 0;                          // Change in strafing encoder rotation
    double leftWheelTickDeltaPrevious = 0;                    // Last change in left encoder rotation
    double rightWheelTickDeltaPrevious = 0;                   // Last change in right encoder rotation
    double strafeWheelTickDeltaPrevious = 0;                  // Last change in strafing encoder rotation
    double xPosition = 0;                                     // Robot's x-position from starting point
    double yPosition = 0;                                     // Robot's y-position from starting point
    double initialHeading = 90;                               // Robot's starting heading
    double robotHeading = 0;                                  // Robot's current heading
    double robotHeadingDelta = 0;                             // Change in robot heading
    double robotHeadingPrevious = 0;                          // Last change in robot heading
    double robotSpeedInFPS;                                   // Robot speed in Feet per second

    // Variables used in Autonomous
    int numberPosition = 1;
    // Variables used in the Teleop drive code
    double angleTest[] = new double[10];         // Last 10 robot headings
    int count = 0;                               // Iterates through the last 10 robot headings
    double sum;                                  // Sum of the last 10 robot headings
    double correct;                              // Angle correction to deal with robot drift
    // Variables used in the Teleop operator code
    double pastEncoderR = 0;
    double pastEncoderL = 0;
    double launchSpeedR;
    double launchSpeedL;
    double newSpeedR;
    double newSpeedL;
    boolean upFlag;
    boolean upFlag2;
    boolean upPersistent;
    boolean upPersistent2;
    boolean downFlag;
    boolean downFlag2;
    boolean downPersistent;
    boolean downPersistent2;
    double wheelSpeedMultiplier = .9;
    double wheelSpeedMultiplier2 = .25;
    // CONSTANTS
    double ENCODER_CPR = 360;                    // Encoder CPR
    double WHEEL_CURCUMFERENCE = 2.28;           // Wheel circumference of the encoder wheels
    double COUNTS_PER_INCH = ENCODER_CPR / WHEEL_CURCUMFERENCE; // Calculated Counts per Inch on the encoders
    double STARTING_HEADING = 0;                 // Robot Starting Heading
    double STRAFE_WHEEL_OFFSET_CONSTANT = 354;   // Deals with the strafing encoder moving from robot turns
    double TARGET_POSITION_ACCURACY_IN_INCHES = .5; // Accuracy constant for robot travel
    double WAYPOINT_POSITION_ACCURACY_IN_INCHES = 2; // Accuracy constant for robot travel
    double TARGET_HEADING_ACCURACY_IN_DEGREES = 2;   // Accuracy constant for robot turning
    double LAUNCHER_SPEED_R = -1875;
    double LAUNCHER_SPEED_L = 1325;

    public CelebrimborBase(OpMode theOpMode) {
        opMode = theOpMode;

        // INITIALIZE MOTORS AND SERVOS
        opMode.telemetry.addLine("Initalizing output devices (motors, servos)...");
        opMode.telemetry.update();

        motorDriveLF = opMode.hardwareMap.dcMotor.get("motorDriveLF");
        motorDriveLB = opMode.hardwareMap.dcMotor.get("motorDriveLB");
        motorDriveRF = opMode.hardwareMap.dcMotor.get("motorDriveRF");
        motorDriveRB = opMode.hardwareMap.dcMotor.get("motorDriveRB");
        motorLaunchL = opMode.hardwareMap.dcMotor.get("motorLaunchL");
        motorLaunchR = opMode.hardwareMap.dcMotor.get("motorLaunchR");
        motorArm = opMode.hardwareMap.dcMotor.get("motorArm");
        motorCollection = opMode.hardwareMap.dcMotor.get("motorCollection");

        //motorLaunchL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLaunchR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorDriveLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLaunchL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLaunchR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorCollection.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorDriveLF.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDriveLB.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDriveRF.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDriveRB.setDirection(DcMotorSimple.Direction.FORWARD);

        resetEncoders();
        runWithoutEncoders();


        servoGoal = opMode.hardwareMap.crservo.get("servoGoal");
        servoGoal.setPower(0);
        servoCollectionRaise = opMode.hardwareMap.crservo.get("servoCollectionRaise");
        servoLimit = opMode.hardwareMap.servo.get("servoLimit");
        servoLimit.setPosition(0);

        // INITIALIZE SENSORS
        opMode.telemetry.addLine("Initalizing input devices (sensors)...");
        opMode.telemetry.update();

        BNO055IMU.Parameters parameters_IMU = new BNO055IMU.Parameters();
        parameters_IMU.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters_IMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters_IMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters_IMU.loggingEnabled = true;
        parameters_IMU.loggingTag = "IMU";
        parameters_IMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters_IMU);

        timerOpMode = new ElapsedTime();
        timerTravel = new ElapsedTime();
        timerLauncher = new ElapsedTime();
        timerSpeedControl = new ElapsedTime();

        opMode.telemetry.addLine("Initialization Succeeded!");
        opMode.telemetry.update();

    }

    /** Just gonna reset the encoders real quick...*/
    public void resetEncoders(){
        motorDriveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /** ... And now we set them to run without the encoders!
     * (NOTE: they will still return position, but there's no fancy PID control on the encoders.)*/
    public void runWithoutEncoders(){
        motorDriveLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Set power to ALL drivetrain motors. Uses setDrivePowerSides() for simplicity.
     * @param motorPower the power given to each motor
     * */
    public void setDrivePower(double motorPower) {
        setDrivePowerSides(motorPower, motorPower);
    }

    /** Set power to the right and left drivetrain sides individually.
     * Uses setDrivePowerMotors() for simplicity.
     * @param motorPowerL the power given to the left side of the drivetrain
     * @param motorPowerR the power given to the right side of the drivetrain
     * */
    public void setDrivePowerSides(double motorPowerL, double motorPowerR) {
        setDrivePowerMotors(motorPowerL, motorPowerL, motorPowerR, motorPowerR);
    }
    /** Set unique powers to each of the drivetrain motors.
     * @param motorPowerLF the power given to the front-left drive motor
     * @param motorPowerLB the power given to the back-left drive motor
     * @param motorPowerRF the power given to the front-right drive motor
     * @param motorPowerRB the power given to the back-right drive motor
     * */
    public void setDrivePowerMotors(double motorPowerLF, double motorPowerLB, double motorPowerRF, double motorPowerRB) {
        motorDriveLF.setPower(motorPowerLF);
        motorDriveLB.setPower(motorPowerLB);
        motorDriveRF.setPower(motorPowerRF);
        motorDriveRB.setPower(motorPowerRB);
    }

    public void autoLaunch(double time){ //
        timerLauncher.reset();
        while (timerLauncher.seconds() < time){
                if (timerSpeedControl.seconds() > .05){
                    launchSpeedR = (motorLaunchR.getCurrentPosition() - pastEncoderR)/.05;
                    launchSpeedL = (motorLaunchL.getCurrentPosition() - pastEncoderL)/.05;
                    timerSpeedControl.reset();
                    pastEncoderR = motorLaunchR.getCurrentPosition();
                    pastEncoderL = motorLaunchL.getCurrentPosition();
                }
            //fancy math stuff
            newSpeedR = (LAUNCHER_SPEED_R + launchSpeedR) * .00005; //multiply error by a constant
            newSpeedL = (LAUNCHER_SPEED_L - launchSpeedL) * .0005;

            motorLaunchL.setPower(wheelSpeedMultiplier2 + newSpeedL); //add additional speed proportionally to how far off it is from the ideal speed
            motorLaunchR.setPower(-wheelSpeedMultiplier - newSpeedR);
        }

    }

    /* =======================AUTONOMOUS EXCLUSIVE METHODS========================= */
    /** Simplifies the code without having to use ((LinearOpMode)opMode).sleep(ms) all the time
     * @param ms how many milliseconds we want to pause for.
     * */
    public void sleep(long ms) {
        if(((LinearOpMode)opMode).opModeIsActive()){
            ((LinearOpMode)opMode).sleep(ms);
        }
    }

    /** Another simplification function, this one checks if the robot is initializing.
     * @return whether or not we have pressed init on the driver station.
     * */
    boolean isInitialized() {
        return !((LinearOpMode)opMode).isStarted() && !((LinearOpMode)opMode).isStopRequested();
    }

    /** We use the driver’s gamepad controller to select autonomous options.
     * The drive team can select the alliance color, parking preference, and whether to pull the
     * foundation, deliver the Skystones, or neither. It then displays the results while we wait
     * for the match to begin.*/
    public void selection() {
        // What alliance color are we? (By the way, that loop is a do-while loop. It's like a while
        // loop, but it always runs at least once because of how it's set up)
        opMode.telemetry.addLine("Awaiting Autonomous Selection...");
        opMode.telemetry.addData("For blue alliance, press", "X");
        opMode.telemetry.addData("For red alliance, press", "B");
        opMode.telemetry.update();

        do {
            sleep(50);
            if(opMode.gamepad1.x) allianceColor = "BLUE";
            if(opMode.gamepad1.b) allianceColor = "RED";
        } while(!opMode.gamepad1.x && !opMode.gamepad1.b && isInitialized());

        /*// Parking Preference?
        opMode.telemetry.addLine("Awaiting Autonomous Selection...");
        opMode.telemetry.addData("To park inside, press", "Y");
        opMode.telemetry.addData("To park outside, press", "A");
        opMode.telemetry.update();
        do {
            sleep(50);
            if(opMode.gamepad1.y) parkingPreference = "INSIDE";
            if(opMode.gamepad1.a) parkingPreference = "OUTSIDE";
        } while(!opMode.gamepad1.y && !opMode.gamepad1.a && isInitialized());
        // Autonomous Strategy?
        opMode.telemetry.addLine("Awaiting Autonomous Selection...");
        opMode.telemetry.addData("To score the second wobble goal, press", "B");
        opMode.telemetry.addData("To just park, press", "START");
        opMode.telemetry.update();
        do {
            sleep(50);
            if(opMode.gamepad1.b) secondWobbleGoal = true;
        } while(!opMode.gamepad1.x && !opMode.gamepad1.b && !opMode.gamepad1.start && isInitialized());*/

        // Display the results.
        opMode.telemetry.addData("All Set! Just press PLAY when ready!", "D");
        opMode.telemetry.addData("Current Alliance", allianceColor);
        opMode.telemetry.addData("Parking Preference", parkingPreference);
        opMode.telemetry.update();
    }

    /** Intakes the motors to be used by each of the encoders, and then updates robot position based
     * on how much change has occured in the motor wheels and robot heading.
     * @param encoderLeft usually uses motorDriveLB
     * @param encoderRight usually uses motorDriveRB
     * @param encoderStrafe usually uses motorDriveLF
     * */
    public void updateOdometry(DcMotor encoderLeft, DcMotor encoderRight, DcMotor encoderStrafe) {
        // STEP 1: Calculate the Delta (change) of the ticks and robot heading since last update
        leftWheelTickDelta = -(encoderLeft.getCurrentPosition() - leftWheelTickDeltaPrevious);
        rightWheelTickDelta = (encoderRight.getCurrentPosition() - rightWheelTickDeltaPrevious);
        strafeWheelTickDelta = -(encoderStrafe.getCurrentPosition() - strafeWheelTickDeltaPrevious);

        leftWheelTickDeltaPrevious = encoderLeft.getCurrentPosition();
        rightWheelTickDeltaPrevious = encoderRight.getCurrentPosition();
        strafeWheelTickDeltaPrevious = encoderStrafe.getCurrentPosition();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotHeading = angles.firstAngle + initialHeading;
        robotHeadingDelta = robotHeading - robotHeadingPrevious;

        robotHeadingPrevious = robotHeading;

        // STEP 2: Refine the changes to get the vertical and horizontal changes in inches
        double leftDistance = leftWheelTickDelta / COUNTS_PER_INCH;
        double rightDistance = rightWheelTickDelta / COUNTS_PER_INCH;
        double verticalDistance = (leftDistance + rightDistance) / 2;
        double horizontalDistance = (strafeWheelTickDelta - (Math.toRadians(robotHeadingDelta) / STRAFE_WHEEL_OFFSET_CONSTANT)) / COUNTS_PER_INCH;

        // STEP 3: Apply Trigonometry to the vertical and horizontal changes and update our position
        // Position Update for the vertical encoder wheels
        xPosition += verticalDistance * Math.cos(Math.toRadians(robotHeading));
        yPosition += verticalDistance * Math.sin(Math.toRadians(robotHeading));
        // Position Update for the horizontal encoder wheel
        xPosition += horizontalDistance * Math.sin(Math.toRadians(robotHeading));
        yPosition -= horizontalDistance * Math.cos(Math.toRadians(robotHeading));

        opMode.telemetry.addData("Robot Speed in Feet per Second", robotSpeedInFPS);
        opMode.telemetry.addData("encoderLeft: " , encoderLeft.getCurrentPosition());
        opMode.telemetry.addData("encoderStrafe: " , encoderStrafe.getCurrentPosition());
        opMode.telemetry.addData("encoderRight: " , encoderRight.getCurrentPosition());
        opMode.telemetry.addData("xPosition: ", xPosition);
        opMode.telemetry.addData("yPosition: ", yPosition);
        opMode.telemetry.update();
    }

    /** Tells the robot to travel to a certain position and heading on the field while constantly
     * updating position and heading.
     * @param xTarget the x-value of the target position relative to starting position in inches.
     * @param yTarget the y-value of the target position relative to starting position in inches.
     * @param lockedHeading what robot heading we want to end up at
     * @param targetAccuracy how close we want to be to the target before we stop.
     * @param powerBoost Optional parameter for cases when we need lots of pushing power
     *                   (*stares at the foundation*)
     * */
    void travelToPosition(double xTarget, double yTarget, double lockedHeading, double targetAccuracy, double... powerBoost) {
        updateOdometry(motorDriveLB, motorDriveRB, motorDriveLF);
        double offsetHeading = lockedHeading - robotHeading;
        double xDelta = -xTarget + xPosition;
        double yDelta = yTarget - yPosition;

        while(((Math.abs(xDelta) > targetAccuracy ||
                Math.abs(yDelta) > targetAccuracy) ||
                (Math.abs(offsetHeading) > TARGET_HEADING_ACCURACY_IN_DEGREES) &&
                        ((LinearOpMode)opMode).opModeIsActive())) {
            updateOdometry(motorDriveLB, motorDriveRB, motorDriveLF);
            xDelta = -xTarget + xPosition;
            yDelta = yTarget - yPosition;
            offsetHeading = lockedHeading - robotHeading;
            offsetHeading += offsetHeading > 120 ? -360 :
                    offsetHeading < -240 ? 360 : 0;

            double desiredSpeedLF = 0;
            double desiredSpeedLB = 0;
            double desiredSpeedRF = 0;
            double desiredSpeedRB = 0;

            if(Math.abs(xDelta) > targetAccuracy ||
                    Math.abs(yDelta) > targetAccuracy) {
                // Determines wheel power for each motor based on our base motor power and target X, Y, and Theta values
                double angleDirection = Math.atan2(yDelta, xDelta) + Math.toRadians(robotHeading) - Math.toRadians(initialHeading);
                double powerMod = powerBoost.length > 0 ? powerBoost[0] : 0; // For when we need heavy duty pushing power
                double powerCurve = Math.sqrt(Math.abs(Math.hypot(xDelta, yDelta))/80);
                double drivingPower = Range.clip(powerCurve + .1 + powerMod, 0, 0.55);
                double wheelTrajectory = angleDirection - (Math.PI/4);

                desiredSpeedLF -= (drivingPower * (Math.cos(wheelTrajectory))) * Math.sqrt(2);
                desiredSpeedLB -= (drivingPower * (Math.sin(wheelTrajectory))) * Math.sqrt(2);
                desiredSpeedRF += (drivingPower * -(Math.sin(wheelTrajectory))) * Math.sqrt(2);
                desiredSpeedRB += (drivingPower * -(Math.cos(wheelTrajectory))) * Math.sqrt(2);
            }
            // Correct for robot drifting
            if(Math.abs(offsetHeading) > TARGET_HEADING_ACCURACY_IN_DEGREES) {
                double turnMod = Range.clip(Math.toRadians(offsetHeading * 1.5), -0.5, 0.5);
                desiredSpeedLF = Range.clip(desiredSpeedLF + turnMod, -1, 1);
                desiredSpeedLB = Range.clip(desiredSpeedLB + turnMod, -1, 1);
                desiredSpeedRF = Range.clip(desiredSpeedRF - turnMod, -1, 1);
                desiredSpeedRB = Range.clip(desiredSpeedRB - turnMod, -1, 1);
            }
            setDrivePowerMotors(-desiredSpeedLF, -desiredSpeedLB, desiredSpeedRF, desiredSpeedRB);
        }
        setDrivePower(0);
        setDrivePower(0);
        imuTurn(offsetHeading);
    }

    /** The IMU Turn we used last year. Not as complicated as the encoder drive, yet more
     * complicated at the same time. Enjoy.
     * @param degreesToTurn how many degrees we want to turn. And no, I'm not a radian guy.
     * */
    private void imuTurn(double degreesToTurn) {
        updateOdometry(motorDriveLB, motorDriveRB, motorDriveLF);
        double currentHeading = angles.firstAngle + 73;
        double targetHeading = degreesToTurn + currentHeading;
        targetHeading += targetHeading > 360 ? -360 :
                targetHeading < 0 ? 360 : 0;

        timerTravel.reset();
        while (Math.abs(degreesToTurn) > 2 && ((LinearOpMode)opMode).opModeIsActive() && timerTravel.seconds() <= 2) {
            updateOdometry(motorDriveLB, motorDriveRB, motorDriveLF);
            currentHeading = angles.firstAngle + 73;
            degreesToTurn = targetHeading - currentHeading;

            double TURN_POWER = Range.clip(Math.signum(degreesToTurn) * (0.2 + (Math.abs(degreesToTurn) / 270)), -0.35, 0.35);
            setDrivePowerSides(-TURN_POWER, -TURN_POWER);
        }
        setDrivePower(0);
    }

    public void determineTargetZone(int positionNumber){
        numberPosition = positionNumber;    //Sets the position determined in the autonomous class to the position variable in the base class
    }

    public void autonomousPath(){
        if (allianceColor == "RED"){
            if (numberPosition == 0){
                servoGoal.setPower(1);      //Grab Wobble Goal
                sleep(750);
                servoGoal.setPower(0);
                travelToPosition(0, 17, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                imuTurn(-45);
                travelToPosition(27, 50, 45, TARGET_POSITION_ACCURACY_IN_INCHES);
                imuTurn(45);
                travelToPosition(28, 79, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                servoGoal.setPower(-1);     //Release Wobble Goal in Target Zone
                sleep(750);
                servoGoal.setPower(0);
                travelToPosition(28, 73, 90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(51, 65, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                servoCollectionRaise.setPower(-1);      //Release Collection
                sleep(500);
                servoCollectionRaise.setPower(0);
                servoLimit.setPosition(0);      //Spin up Launcher
                motorLaunchL.setPower(50);
                motorLaunchR.setPower(-40);
                sleep(1000);
                servoLimit.setPosition(1);
                sleep(500);
                motorCollection.setPower(100);      //Begin Launching Rings
                sleep(4000);
                motorLaunchL.setPower(0);
                motorLaunchR.setPower(0);
                motorCollection.setPower(0);
                servoLimit.setPosition(0);
                travelToPosition(52, 78, 90, TARGET_POSITION_ACCURACY_IN_INCHES);       //Park on Launch Line
            }
            if (numberPosition == 1){
                servoGoal.setPower(1);      //Grab Wobble Goal
                sleep(750);
                servoGoal.setPower(0);
                travelToPosition(0, 17, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                imuTurn(-45);
                travelToPosition(27, 50, 45, TARGET_POSITION_ACCURACY_IN_INCHES);
                imuTurn(45);
                travelToPosition(52, 105, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                servoGoal.setPower(-1);     //Release Wobble Goal in Target Zone
                sleep(750);
                servoGoal.setPower(0);
                travelToPosition(52, 70, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                servoCollectionRaise.setPower(-1);      //Release Collection
                sleep(500);
                servoCollectionRaise.setPower(0);
                servoLimit.setPosition(0);
                motorLaunchL.setPower(25);      //Spin up Launcher
                motorLaunchR.setPower(-90);
                sleep(2000);
                servoLimit.setPosition(1);
                sleep(500);
                motorCollection.setPower(90);      //Begin Launching Rings
                autoLaunch(4);       //sleep(4000);
                motorLaunchL.setPower(0);
                motorLaunchR.setPower(0);
                motorCollection.setPower(0);
                servoLimit.setPosition(0);
                travelToPosition(52, 78, 90, TARGET_POSITION_ACCURACY_IN_INCHES);       //Park on Launch Line
            }
            if (numberPosition == 4) {
                servoGoal.setPower(1);      //Grab Wobble Goal
                sleep(750);
                servoGoal.setPower(0);
                travelToPosition(0, 17, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                imuTurn(-45);
                travelToPosition(29, 51, 45, TARGET_POSITION_ACCURACY_IN_INCHES);
                imuTurn(45);
                travelToPosition(27, 122, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                servoGoal.setPower(-1);     //Release Wobble Goal in Target Zone
                sleep(750);
                servoGoal.setPower(0);
                travelToPosition(28, 112, 90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(50, 70, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                servoCollectionRaise.setPower(-1);      //Release Collection
                sleep(500);
                servoCollectionRaise.setPower(0);
                servoLimit.setPosition(0);      //Spin up Launcher
                motorLaunchL.setPower(30);
                motorLaunchR.setPower(-20);
                sleep(1000);
                servoLimit.setPosition(1);
                sleep(500);
                motorCollection.setPower(100);      //Begin Launching Rings
                sleep(4000);
                motorLaunchL.setPower(0);
                motorLaunchR.setPower(0);
                motorCollection.setPower(0);
                servoLimit.setPosition(0);
                travelToPosition(52, 78, 90, TARGET_POSITION_ACCURACY_IN_INCHES);       //Park on Launch Line
            }
        }
    }

    /** Wait for the end of autonomous to store our heading.*/
    public void waitForEnd(){
        while (timerOpMode.seconds() <29 && ((LinearOpMode)opMode).opModeIsActive()){ // Yes. It is, in fact, an empty loop.
        }
    }

    /** At the end of the autonomous period, the robot stores the heading of the robot in a file
     * setting. The robot will retrieve this heading at the start of autonomous to be used in the
     * field-centric driver controls. */
    public void storeHeading(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        ReadWriteFile.writeFile(headingFile, String.valueOf(-angles.firstAngle));
        opMode.telemetry.addData("headingFile", "" + ReadWriteFile.readFile(headingFile)); //Store robot heading to phone
        opMode.telemetry.update();
    }

    /* =======================TELEOP METHODS========================= */

    /** At the start of the program, the robot retrieves the robot’s heading stored in a file
     * setting on the phone during Autonomous. This is used in our Field-Centric Drive Control
     * of the robot.*/
    public void retrieveHeading() {
        try {
            STARTING_HEADING = Double.parseDouble(ReadWriteFile.readFile(headingFile));
            opMode.telemetry.addData("Heading retrieved", STARTING_HEADING);
            opMode.telemetry.update();
        } catch(Exception exc) {
            opMode.telemetry.addData("ERROR", "Couldn't read headingFile; will set startingHeading to 0.");
            opMode.telemetry.update();
            STARTING_HEADING = 0;
        }
    }

    /** In the event that the teleop retrieves a faulty robot heading without running autonomous
     * first, the driver can simply realign the robot with the field and set that heading as the
     * starting heading, satisfying the Field-Centric Drive Control of the robot.*/
    public void resetHeading(){ // Resets the imu heading by adding/subtracting from itself
        if (opMode.gamepad1.b){ // In case somethine goes wrong, driver can reposition the robot and reset the heading during teleop
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            if (angles.firstAngle > 0){
                angles.firstAngle -= 2 * angles.firstAngle;
            }
            else {
                angles.firstAngle += 2 * angles.firstAngle;
            }
            STARTING_HEADING = angles.firstAngle;
        }
    }

    /** Updates drivetrain motor powers based on the driver's right and left joysticks.
     * Uses Field-centric Driver control for easier driving.*/
    public void updateDriveTrain() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double speed = Math.hypot(opMode.gamepad1.left_stick_x, opMode.gamepad1.left_stick_y);
        double angle = Math.atan2(opMode.gamepad1.left_stick_y, opMode.gamepad1.left_stick_x) - (Math.PI/4);
        angle += angles.firstAngle - (Math.PI)/2 + STARTING_HEADING;
        double turnPower = opMode.gamepad1.right_stick_x;

        if(turnPower == 0){
            if (count < 10) {
                angleTest[count] = angle;
                angleTest[count] = angle;
                count++;
            }
            else if (count >= 10){
                sum = (angleTest[1] + angleTest[2] + angleTest[3] + angleTest[4] + angleTest[0] + angleTest[5] + angleTest[6] + angleTest[7] + angleTest[8] + angleTest[9])/10;
                if(sum > angle){
                    correct = sum - angle;
                    angle = angle + correct;
                }
                else if(angle > sum){
                    correct = angle - sum;
                    angle = angle - correct;
                }
                count = 0;

            }
        }

        motorDriveLF.setPower(((speed * -(Math.sin(angle)) + turnPower)));
        motorDriveLB.setPower(((speed * -(Math.cos(angle)) + turnPower)));
        motorDriveRF.setPower(((speed * (Math.cos(angle))) + turnPower));
        motorDriveRB.setPower(((speed * (Math.sin(angle))) + turnPower));

    }

    public void unlatchCollection(){
        if (opMode.gamepad1.right_bumper){
            servoCollectionRaise.setPower(-1);
        }
        else if (opMode.gamepad1.left_bumper){
            servoCollectionRaise.setPower(1);
        }
    }

    //===========================================Operator Controls go here:=======================================

    public void controlCollection(){
        motorCollection.setPower(-opMode.gamepad2.left_stick_y);
    }
    public void controlWobbleGoal(){
        motorArm.setPower(-opMode.gamepad2.right_stick_y);

        if (opMode.gamepad2.left_bumper){
            servoGoal.setPower(-1);
        }
        else if (opMode.gamepad2.right_bumper){
            servoGoal.setPower(1);
        }
        else {
            servoGoal.setPower(0);
        }
    }

    public void dpadStuffs(){
        if (opMode.gamepad2.dpad_up){
            upFlag = true;
        } else {
            upFlag = false;
            upPersistent = false;
        }
        if (upFlag && !upPersistent) {
            if (wheelSpeedMultiplier < 1){wheelSpeedMultiplier += .05;}
            upPersistent = true;
        }

        if (opMode.gamepad2.dpad_down){
            downFlag = true;
        } else {
            downFlag = false;
            downPersistent = false;
        }
        if (downFlag && !downPersistent) {
            if (wheelSpeedMultiplier > .1){wheelSpeedMultiplier -= .05;}
            downPersistent = true;
        }


        if (opMode.gamepad2.dpad_right){
            upFlag2 = true;
        } else {
            upFlag2 = false;
            upPersistent2 = false;
        }
        if (upFlag2 && !upPersistent2) {
            if (wheelSpeedMultiplier2 < 1){wheelSpeedMultiplier2 += .05;}
            upPersistent2 = true;
        }

        if (opMode.gamepad2.dpad_left){
            downFlag2 = true;
        } else {
            downFlag2 = false;
            downPersistent2 = false;
        }
        if (downFlag2 && !downPersistent2) {
            if (wheelSpeedMultiplier2 > .1){wheelSpeedMultiplier2 -= .05;}
            downPersistent2 = true;
        }
    }

    public void controlLauncher(){
        if (opMode.gamepad2.right_trigger > .1){
            if (timerSpeedControl.seconds() > .05){
                launchSpeedR = (motorLaunchR.getCurrentPosition() - pastEncoderR)/.05; // Encoder Ticks per Second
                launchSpeedL = (motorLaunchL.getCurrentPosition() - pastEncoderL)/.05;
                timerSpeedControl.reset();
                pastEncoderR = motorLaunchR.getCurrentPosition();
                pastEncoderL = motorLaunchL.getCurrentPosition();
            }
            //fancy math stuff
            newSpeedR = (LAUNCHER_SPEED_R + launchSpeedR) * .00005; //multiply error by a constant
            newSpeedL = (LAUNCHER_SPEED_L - launchSpeedL) * .0005;

            motorLaunchL.setPower(opMode.gamepad2.right_trigger * wheelSpeedMultiplier2 + newSpeedL); //add additional speed proportionally to how
            motorLaunchR.setPower(-opMode.gamepad2.right_trigger * wheelSpeedMultiplier - newSpeedR);
        }
        else if (opMode.gamepad2.left_trigger > .1){
            motorLaunchL.setPower(-opMode.gamepad2.left_trigger * wheelSpeedMultiplier2);
            motorLaunchR.setPower(opMode.gamepad2.left_trigger * wheelSpeedMultiplier);
        }
        else {
            motorLaunchL.setPower(0);
            motorLaunchR.setPower(0);
        }
        if (opMode.gamepad2.a){
            servoLimit.setPosition(0);
        }
        if (opMode.gamepad2.y){
            servoLimit.setPosition(1);
        }
    }

    /** All telemetry readings are posted down here.*/
    public void postTelemetry() {
        opMode.telemetry.addData("wheelSpeedMultiplier", wheelSpeedMultiplier);
        opMode.telemetry.addData("wheelSpeedMultiplier2", wheelSpeedMultiplier2);
        opMode.telemetry.addData("launchSpeedR", launchSpeedR);
        opMode.telemetry.addData("launchSpeedL", launchSpeedL);
        opMode.telemetry.addData("launcherR", newSpeedR);
        opMode.telemetry.addData("launcherL", newSpeedL);
        //updateOdometry(motorDriveLB, motorDriveRB, motorDriveLF);
        //opMode.telemetry.addLine();
        opMode.telemetry.addData("Running for...", timerOpMode.seconds());
        opMode.telemetry.addLine();
        //opMode.telemetry.addData("RobotHeading", angles.firstAngle + STARTING_HEADING);
        //opMode.telemetry.update();
    }
}