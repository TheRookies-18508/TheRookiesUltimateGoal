package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvTrackerApiPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMapTheRookies;
import com.qualcomm.robotcore.hardware.HardwareMap;


import com.acmerobotics.dashboard.FtcDashboard;



@Autonomous
public class OpenCV extends HardwareMapTheRookies
{

    OpenCvCamera webcam;

    SkystoneDeterminationPipeline pipeline;

    @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");



        FtcDashboard.getInstance().startCameraStream(webcam, 20);


        pipeline = new SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */

                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
        init(hardwareMap);
        leggo();
        waitForStart();

        while (opModeIsActive())

        {


            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();
            sleep(500);
            telemetry.addData("Number of Rings", pipeline.position);
            telemetry.update();
            //TESTING_SPEED = 3600;
            if ((pipeline.position.toString()) == "NONE" && (pipeline.getAnalysis() > 0)){
                telemetry.addData("Ring Pos", "NONE");
                telemetry.update();
                shootTop();
                moveForwardEncoder(1, 3);//.7
                sleep(1000);
                strafeRightEncoder(1, 12);//.5
                sleep(1000);
                gyroTurn(.2,3);
                sleep(1000);


                servoArm.setPosition(.7);
                sleep(500);
                hold();
                sleep(1000);

                strafeLeft(1,15);//.5
                servoArm.setPosition(0);
                sleep(1000);
                gyroTurn(.5,160);
                break;
            }
            else if ((pipeline.position.toString()) == "ONE" && (pipeline.getAnalysis()>0)){
                telemetry.addData("Ring Pos", "ONE");
                telemetry.update();
                shootTop();
                moveForwardEncoder(.4, 22);//.4
                sleep(1000);
                strafeLeft(.4,12);//.4
                sleep(1000);
                dropWobble_park();

                break;
            }
            if ((pipeline.position.toString()) == "FOUR" && (pipeline.getAnalysis()>0)){
                telemetry.addData("Ring Pos", "four");
                telemetry.update();
                shootTop();
                moveForwardEncoder(.7, 35);
                sleep(1000);
                strafeRightEncoder(.5, 17);
                gyroTurn(.4, 49);
                sleep(1000);
                sleep(1000);


                servoArm.setPosition(.7);
                sleep(500);
                hold();
                sleep(500);

                moveForwardEncoder(.5, 8);
                sleep(100);
                strafeLeft(1,49);

//                gyroTurn(.2,-95);
                servoArm.setPosition(0);
                sleep(500);

                break;
            }


            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(170,165);

        static final int REGION_WIDTH = 30;
        static final int REGION_HEIGHT = 65;

        final int FOUR_RING_THRESHOLD = 140;
        final int ONE_RING_THRESHOLD = 130;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

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

            //position = RingPosition.FOUR; // Record our analysis
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
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }

    }



}