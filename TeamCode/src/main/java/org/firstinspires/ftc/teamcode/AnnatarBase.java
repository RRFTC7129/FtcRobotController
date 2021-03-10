package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.openftc.easyopencv.OpenCvCamera;

import java.io.File;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

class AnnatarBase {
    OpMode opMode;

    //Declare all devices
    DcMotor motorDriveLF, motorDriveLB, motorDriveRF, motorDriveRB, motorArm, motorCollection, motorLaunch, motorTransfer;
    CRServo servoGoal, servoCollectionRaise, servoLaunchAngle, servoLaunchAngle2;
    BNO055IMU imu;                   // IMU Gyro itself
    Orientation angles;              // IMU Gyro's Orienting
    //External Sensors
    DistanceSensor sensorTransfer;
    DistanceSensor sensorLauncher;
    //Camera
    OpenCvCamera webcam;
    // Stores the robot's heading at the end of autonomous to be used in teleop
    File headingFile = AppUtil.getInstance().getSettingsFile("headingFile");
    // Timers
    ElapsedTime timerOpMode;         // Tells us how long the Opmode is running since it started.
    ElapsedTime timerTravel;         // Used to cancel a robot turn in case we lock up.
    ElapsedTime timerLauncher;       // Used to stop the firing function if it's taking too long
    ElapsedTime timerSpeedControl;   // Used in the proportional speed control for the launcher
    ElapsedTime timerTransfer;

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
    double finalSpeed; //Power applied to the launcher motor
    // Variables used in the Teleop drive code
    double angleTest[] = new double[10];         // Last 10 robot headings
    int count = 0;                               // Iterates through the last 10 robot headings
    double sum;                                  // Sum of the last 10 robot headings
    double correct;                              // Angle correction to deal with robot drift
    // Variables used in the Teleop operator code
    int transferTarget;
    double pastEncoderP = 0;
    double launchSpeed;
    double newSpeed;
    double wheelSpeedMultiplier = 1;
    boolean raiseAngle = false;
    boolean lowerAngle = false;
    boolean autoTransfer = false;
    boolean upFlag;
    boolean upPersistent;
    boolean downFlag;
    boolean downPersistent;
    boolean launchForwards = false;
    boolean powerShots = false;
    boolean disableAutoTransfer = false;
    String angleTarget;
    int finalTransferTarget;
    double targetPosition1;
    double target;
    double currentTransferPosition;
    double rotation = 1736;
    // CONSTANTS
    double ENCODER_CPR = 360;                    // Encoder CPR
    double WHEEL_CURCUMFERENCE = 2.28;           // Wheel circumference of the encoder wheels
    double COUNTS_PER_INCH = ENCODER_CPR / WHEEL_CURCUMFERENCE; // Calculated Counts per Inch on the encoders
    double STARTING_HEADING = 0;                 // Robot Starting Heading
    double STRAFE_WHEEL_OFFSET_CONSTANT = 354;   // Deals with the strafing encoder moving from robot turns
    double TARGET_POSITION_ACCURACY_IN_INCHES = .5; // Accuracy constant for robot travel
    double WAYPOINT_POSITION_ACCURACY_IN_INCHES = 2; // Accuracy constant for robot travel
    double TARGET_HEADING_ACCURACY_IN_DEGREES = 2;   // Accuracy constant for robot turning
    double LAUNCHER_SPEED = 4000; // The target speed for the launcher flywheel
    double POWER_SHOT_ANGLE_MAX = 21000; // The angle required for hitting the power shots
    double POWER_SHOT_ANGLE_MIN = 15000;
    double HIGH_GOAL_ANGLE_MAX = 30000; // The angle required for launching into the high goal
    double HIGH_GOAL_ANGLE_MIN = 22000;
    double LAUNCH_ANGLE_LIMIT = 1000000; // Limit the maximum launch angle so we don't overextend
    //State Machine
    private enum TransferState{                     // Automatic Transfer System (ATS) steps
        IDLE, MEASURING, RAISING
    }
    private enum LaunchAngleState{
        IDLE, MOVING, STOPPING
    }
    TransferState transfer = TransferState.IDLE;     // What step is the ATS on?
    LaunchAngleState launchAngle = LaunchAngleState.IDLE;

    public AnnatarBase(OpMode theOpMode) {
        opMode = theOpMode;

        // INITIALIZE MOTORS AND SERVOS
        opMode.telemetry.addLine("Initalizing output devices (motors, servos)...");
        opMode.telemetry.update();

        motorDriveLF = opMode.hardwareMap.dcMotor.get("motorDriveLF");
        motorDriveLB = opMode.hardwareMap.dcMotor.get("motorDriveLB");
        motorDriveRF = opMode.hardwareMap.dcMotor.get("motorDriveRF");
        motorDriveRB = opMode.hardwareMap.dcMotor.get("motorDriveRB");
        motorLaunch = opMode.hardwareMap.dcMotor.get("motorLaunch");
        motorArm = opMode.hardwareMap.dcMotor.get("motorArm");
        motorCollection = opMode.hardwareMap.dcMotor.get("motorCollection");
        motorTransfer = opMode.hardwareMap.dcMotor.get("motorTransfer");

        servoLaunchAngle = opMode.hardwareMap.crservo.get("servoLaunchAngle");
        servoLaunchAngle2 = opMode.hardwareMap.crservo.get("servoLaunchAngle2");
        servoGoal = opMode.hardwareMap.crservo.get("servoGoal");
        servoGoal.setPower(0);

        motorDriveLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorCollection.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTransfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorDriveLF.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDriveLB.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDriveRF.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDriveRB.setDirection(DcMotorSimple.Direction.FORWARD);

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

        sensorTransfer = opMode.hardwareMap.get(DistanceSensor.class, "sensorTransfer");
        sensorLauncher = opMode.hardwareMap.get(DistanceSensor.class, "sensorLauncher");

        timerOpMode = new ElapsedTime();
        timerTravel = new ElapsedTime();
        timerLauncher = new ElapsedTime();
        timerSpeedControl = new ElapsedTime();
        timerTransfer = new ElapsedTime();

        opMode.telemetry.addLine("Initialization Succeeded!");
        opMode.telemetry.update();

    }

    /** Just gonna reset the encoders real quick...*/
    public void resetEncoders(){
        motorDriveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /** ... And now we set them to run without the encoders!
     * (NOTE: they will still return position, but there's no fancy PID control on the encoders.)*/
    public void runWithoutEncoders(){
        motorDriveLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLaunch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        // Power Shots?
        opMode.telemetry.addLine("Awaiting Autonomous Selection...");
        opMode.telemetry.addData("For power shots, press", "Y");
        opMode.telemetry.addData("For high goal, press", "A");
        opMode.telemetry.update();
        do {
            sleep(50);
            if(opMode.gamepad1.y) powerShots = true;
            if(opMode.gamepad1.a) powerShots = false;
        } while(!opMode.gamepad1.y && !opMode.gamepad1.a && isInitialized());

        // Display the results.
        opMode.telemetry.addData("All Set! Just press PLAY when ready!", "D");
        opMode.telemetry.addData("Current Alliance", allianceColor);
        opMode.telemetry.addData("Parking Preference", parkingPreference);
        opMode.telemetry.update();
    }

    public void autoLaunch(double time){
        motorLaunch.setPower(1);      //Spin up Launcher
        sleep(2000);
        motorTransfer.setPower(-.6);
        timerLauncher.reset();
        //Speed Controller
        while (timerLauncher.seconds() < time){
            if (timerSpeedControl.seconds() > .05){
                launchSpeed = (motorLaunch.getCurrentPosition() - pastEncoderP)/.05;
                timerSpeedControl.reset();
                pastEncoderP = motorLaunch.getCurrentPosition();
            }
            //fancy math stuff
            newSpeed = (LAUNCHER_SPEED + launchSpeed) * .00003; //multiply error by a constant
            finalSpeed = wheelSpeedMultiplier + newSpeed;
            motorLaunch.setPower(finalSpeed + .5); //add additional speed proportionally to how far off it is from the ideal speed
            opMode.telemetry.addData("launchSpeed", launchSpeed);
            opMode.telemetry.addData("newSpeed", newSpeed);
            opMode.telemetry.update();

            if (sensorLauncher.getDistance(DistanceUnit.CM) <= 2){
                sleep(100);
                motorTransfer.setPower(0);
                sleep(1000);
                motorTransfer.setPower(-.6);
            }
        }

        motorLaunch.setPower(0);
        motorTransfer.setPower(0);
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
        strafeWheelTickDelta = -(-encoderStrafe.getCurrentPosition() - strafeWheelTickDeltaPrevious);

        leftWheelTickDeltaPrevious = encoderLeft.getCurrentPosition();
        rightWheelTickDeltaPrevious = encoderRight.getCurrentPosition();
        strafeWheelTickDeltaPrevious = -encoderStrafe.getCurrentPosition();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //ZYX
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
        opMode.telemetry.addData("encoderStrafe: " , -encoderStrafe.getCurrentPosition());
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

    public void deliverWobbleGoal(){
        if (allianceColor == "RED"){
            if (numberPosition == 0){
                servoLaunchAngle.setPower(1);
                servoLaunchAngle2.setPower(1);
                servoGoal.setPower(-1);
                sleep(500);
                servoGoal.setPower(0);
                motorArm.setPower(.5);
                sleep(350);
                motorArm.setPower(0);
                servoGoal.setPower(1);      //Grab Wobble Goal
                sleep(750);
                servoGoal.setPower(0);
                travelToPosition(-65, 20, 90, WAYPOINT_POSITION_ACCURACY_IN_INCHES, 100);
                travelToPosition(-81, -25, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                servoGoal.setPower(-1);
                sleep(750);
                servoGoal.setPower(0);
                servoLaunchAngle.setPower(0);
                servoLaunchAngle2.setPower(0);
                sleep(1000);
            }
            else if (numberPosition == 1){
                servoLaunchAngle.setPower(1);
                servoLaunchAngle2.setPower(1);
                servoGoal.setPower(-1);
                sleep(500);
                servoGoal.setPower(0);
                motorArm.setPower(.5);
                sleep(350);
                motorArm.setPower(0);
                servoGoal.setPower(1);      //Grab Wobble Goal
                sleep(750);
                servoGoal.setPower(0);
                travelToPosition(-65, 20, 90, WAYPOINT_POSITION_ACCURACY_IN_INCHES, 100);
                travelToPosition(-105, 0, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                servoGoal.setPower(-1);
                sleep(750);
                servoGoal.setPower(0);
                servoLaunchAngle.setPower(0);
                servoLaunchAngle2.setPower(0);
                travelToPosition(-90, 15, 90, WAYPOINT_POSITION_ACCURACY_IN_INCHES, 100);
            }
            else if (numberPosition == 4){
                servoLaunchAngle.setPower(1);
                servoLaunchAngle2.setPower(1);
                servoGoal.setPower(-1);
                sleep(500);
                servoGoal.setPower(0);
                motorArm.setPower(.5);
                sleep(350);
                motorArm.setPower(0);
                servoGoal.setPower(1);      //Grab Wobble Goal
                sleep(750);
                servoGoal.setPower(0);
                travelToPosition(-65, 20, 90, WAYPOINT_POSITION_ACCURACY_IN_INCHES, 100);
                travelToPosition(-128, -25, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                servoGoal.setPower(-1);
                sleep(750);
                servoGoal.setPower(0);
                servoLaunchAngle.setPower(0);
                servoLaunchAngle2.setPower(0);
                travelToPosition(-122, -15, 90, WAYPOINT_POSITION_ACCURACY_IN_INCHES, 100);
            }
        }
    }

    public void launchRings(){
        if (allianceColor == "RED"){
            travelToPosition(-81, -10, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
            travelToPosition(-120, 5, 0, TARGET_POSITION_ACCURACY_IN_INCHES);
            autoLaunch(6.5);
            travelToPosition(-100, -20 , 0, TARGET_POSITION_ACCURACY_IN_INCHES);
        }
    }

    public void launchTest(){
        if (allianceColor != "RED"){
            autoLaunch(7);
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
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS); //AxesOrder.ZYX  //workingish: xyz

        double speed = Math.hypot(opMode.gamepad1.left_stick_x, opMode.gamepad1.left_stick_y);
        double angle = Math.atan2(opMode.gamepad1.left_stick_y, opMode.gamepad1.left_stick_x) - (Math.PI/4); // PI/4
        angle += angles.firstAngle - (Math.PI) + STARTING_HEADING; //angle += angles.firstAngle - (Math.PI)/2 + STARTING_HEADING;
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

    //===========================================Operator Controls go here:=======================================

    public void controlCollection(){
        motorCollection.setPower(opMode.gamepad2.left_stick_y);
    }

    public void autoTransfer(){
        if (opMode.gamepad2.x){ //Manually reset the transfer position
            motorTransfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorTransfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (opMode.gamepad2.dpad_left){
            disableAutoTransfer = true;
        }
        if (opMode.gamepad2.dpad_right){
            disableAutoTransfer = false;
        }
        // Updates the proximity sensor's readings.
        if (sensorTransfer.getDistance(DistanceUnit.INCH) <= .7 && !disableAutoTransfer){
            autoTransfer = true;
        }
        else {
            autoTransfer = false;
        }
        // State machines allow us to run a sequence program within an updating loop.
        switch(transfer.ordinal())  {
            // If there is a ring in the collection
            // and we aren't already lifting a ring,
            // enter the state machine to automatically lift it
            case 0:
                if (autoTransfer && transfer == TransferState.IDLE){
                    transfer = TransferState.MEASURING;
                    timerTransfer.reset();
                }
                break;
            // Lift the ring one rotation
            case 1:
                currentTransferPosition = motorTransfer.getCurrentPosition();
                    transfer = TransferState.RAISING;
                break;
            case 2:
                transferTarget = (int) currentTransferPosition - 730;
                motorTransfer.setTargetPosition(transferTarget);
                motorTransfer.setPower(-.5);
                if (motorTransfer.getCurrentPosition() <= currentTransferPosition - 730){
                    motorTransfer.setPower(0);
                    transfer = TransferState.IDLE;
                    autoTransfer = false;
                }
                break;
        }
    }
    public void controlTransfer() {
            motorTransfer.setPower(-opMode.gamepad2.right_stick_x);

            //Lock the transfer in the ready to collect position
            if (opMode.gamepad2.right_bumper) {
                rotation = 750; //Number of ticks for one full rotation
                currentTransferPosition = motorTransfer.getCurrentPosition();
                targetPosition1 = (currentTransferPosition / rotation);
                target = Math.round(targetPosition1);
                finalTransferTarget = (int) (target * rotation);
                motorTransfer.setTargetPosition(finalTransferTarget);
                motorTransfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorTransfer.setPower(1);
            }
            else {
                motorTransfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    public void controlWobbleGoal(){
        motorArm.setPower(-opMode.gamepad2.right_stick_y * 2);

        if (opMode.gamepad1.left_bumper){
            servoGoal.setPower(-1);
        }
        else if (opMode.gamepad1.right_bumper){
            servoGoal.setPower(1);
        }
        else {
            servoGoal.setPower(0);
        }
    }

    public void flywheelSpeed(){
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
    }

    public void controlLauncher(){
        if (opMode.gamepad2.y){
            launchForwards = true;
        }
        if (opMode.gamepad2.a){
            launchForwards = false;
        }

        if (launchForwards == true){
            if (timerSpeedControl.seconds() > .05){
                launchSpeed = (motorLaunch.getCurrentPosition() - pastEncoderP)/.05; // Encoder Ticks per Second
                timerSpeedControl.reset();
                pastEncoderP = motorLaunch.getCurrentPosition();
            }
            //fancy math stuff
            newSpeed = (LAUNCHER_SPEED - launchSpeed) * .00003; //multiply error by a constant
            finalSpeed = newSpeed + wheelSpeedMultiplier;

            //motorLaunch.setPower(finalSpeed + .2); //add additional speed proportionally to how far off we are
            motorLaunch.setPower(1 * wheelSpeedMultiplier);
        }
        else if (opMode.gamepad2.left_trigger > .1){
            motorLaunch.setPower(-1 * wheelSpeedMultiplier);
        }
        else {
            motorLaunch.setPower(0);
        }
    }

    public void launchAngle(){
        //Change the angle of the launcher
        if (opMode.gamepad1.dpad_up && motorArm.getCurrentPosition() < LAUNCH_ANGLE_LIMIT){
            servoLaunchAngle.setPower(1);
            servoLaunchAngle2.setPower(1);
            raiseAngle = false;
            lowerAngle = false;
        }
        else if (opMode.gamepad1.dpad_down){
            servoLaunchAngle.setPower(-1);
            servoLaunchAngle2.setPower(-1);
            raiseAngle = false;
            lowerAngle = false;
        }
        else if (!opMode.gamepad1.dpad_down && !opMode.gamepad1.dpad_up && !raiseAngle && !lowerAngle){
            servoLaunchAngle.setPower(0);
            servoLaunchAngle2.setPower(0);
        }
        if (opMode.gamepad1.dpad_left && motorArm.getCurrentPosition() < HIGH_GOAL_ANGLE_MAX){
            raiseAngle = true;
            lowerAngle = false;
            angleTarget = "HIGH";
        }
        else if (opMode.gamepad1.dpad_left && motorArm.getCurrentPosition() > HIGH_GOAL_ANGLE_MAX){
            raiseAngle = false;
            lowerAngle = true;
            angleTarget = "HIGH";
        }
        else if (opMode.gamepad1.dpad_right && motorArm.getCurrentPosition() < POWER_SHOT_ANGLE_MAX){
            raiseAngle = true;
            lowerAngle = false;
            angleTarget = "POWER";
        }
        else if (opMode.gamepad1.dpad_right && motorArm.getCurrentPosition() > POWER_SHOT_ANGLE_MAX){
            raiseAngle = false;
            lowerAngle = true;
            angleTarget = "POWER";
        }
        // State machines allow us to run a sequence program within an updating loop.
        switch(launchAngle.ordinal())  {
            // If we want to change the angle of the launcher
            // enter the state machine to raise/lower it
            case 0:
                if ((raiseAngle || lowerAngle) && launchAngle == LaunchAngleState.IDLE){
                    launchAngle = LaunchAngleState.MOVING;
                }
                break;
            // Power the servos in the correct direction
            case 1:
                if (raiseAngle == true){
                    servoLaunchAngle.setPower(1);
                    servoLaunchAngle2.setPower(1);
                }
                if (lowerAngle == true){
                    servoLaunchAngle.setPower(-1);
                    servoLaunchAngle.setPower(-1);
                }
                launchAngle = LaunchAngleState.STOPPING;
                break;
            case 2:
                if ((((motorArm.getCurrentPosition() < HIGH_GOAL_ANGLE_MAX) &&
                        (motorArm.getCurrentPosition() > HIGH_GOAL_ANGLE_MIN)) && angleTarget == "HIGH") ||
                        (((motorArm.getCurrentPosition() < POWER_SHOT_ANGLE_MAX) &&
                                (motorArm.getCurrentPosition() > POWER_SHOT_ANGLE_MIN)) && angleTarget == "POWER")){
                    servoLaunchAngle.setPower(0);
                    servoLaunchAngle2.setPower(0);
                    raiseAngle = false;
                    lowerAngle = false;
                    launchAngle = LaunchAngleState.IDLE;
                }
                break;
        }
    }

    /** All telemetry readings are posted down here.*/
    public void postTelemetry() {
        opMode.telemetry.addData("launchSpeed", launchSpeed);
        //opMode.telemetry.addData("newSpeed", newSpeed);
        //updateOdometry(motorDriveLB, motorDriveRB, motorDriveLF);
        //opMode.telemetry.addData("finalSpeed", finalSpeed);
        opMode.telemetry.addData("launchPower", wheelSpeedMultiplier);
        opMode.telemetry.addData("launchAngle", motorArm.getCurrentPosition());
        //opMode.telemetry.addData("transferPosition", motorTransfer.getCurrentPosition());
        //opMode.telemetry.addData("Running for...", timerOpMode.seconds());
        opMode.telemetry.update();
    }
}