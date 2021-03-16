package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
//@Disabled
@Autonomous(name="Annatar Auto", group="")
public class AnnatarAuto extends LinearOpMode {
    OpenCvCamera webcam;
    SkystoneDeterminationPipeline pipeline;
    AnnatarBase base;
    int positionNumber = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        base = new AnnatarBase(this);
        base.selection();
        base.timerOpMode.reset();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);
        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        //webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.MAXIMIZE_EFFICIENCY);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }
        });
        for (int i = 0; i < 4; i++){
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();
            sleep(2000);
        }
        if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.NONE){positionNumber = 0;}
        if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE){positionNumber = 1;}
        if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.FOUR){positionNumber = 4;}
        base.determineTargetZone(positionNumber); //Sets the position determined in the autonomous class to the position variable in the base class
        waitForStart();
        webcam.closeCameraDevice();
        base.resetEncoders();
        base.runWithoutEncoders();
        base.deliverWobbleGoal(); //Deliver the Wobble Goal
        base.launchRings(); //Launch the Rings
        base.waitForEnd(); //Waits for the end before storing the final heading
        base.storeHeading(); //Stores the robot's final heading to be used to correct the teleop field centric code
        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();
    }
    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the Target Zone position
         */
        enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }
        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        /*
         * The core values which define the location and size of the sample regions
         */
        //x:130 y:210
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(65 ,178);
        // width: 40 height: 40
        static final int REGION_WIDTH = 130;
        static final int REGION_HEIGHT = 65;
        final int FOUR_RING_THRESHOLD = 133;
        final int ONE_RING_THRESHOLD = 128;
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y - REGION_HEIGHT);
        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;
        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.ONE;
        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }
        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }
        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);
            avg1 = (int) Core.mean(region1_Cb).val[0];
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    1); // Negative thickness means solid fill
            return input;
        }
        public int getAnalysis()
        {
            return avg1;
        }
    }
}