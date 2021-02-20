package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autonomous/TensorflowObjectTest_NEEDTOTEST.java
<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autonomous/TensorflowObjectTest.java
import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMapTheRookies;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;


import com.acmerobotics.dashboard.FtcDashboard;



@Autonomous
public class TensorflowObjectTest extends HardwareMapTheRookies {
    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/model_unquant.tflite";
    private static final String LABEL_FIRST_ELEMENT = "0_rings";
    private static final String LABEL_SECOND_ELEMENT = "1_ring";
    private static final String LABEL_THIRD_ELEMENT = "4_rings";
    OpenCvCamera webcam;
=======

=======

>>>>>>> c68f9a4510d593887e8b1ebf4e1f4da5845d07fa:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autonomous/TensorflowObjectTest.java
/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp(name = "Concept: TensorFlow Object Detection Webcam")
public class TensorflowObjectTest_NEEDTOTEST extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autonomous/TensorflowObjectTest_NEEDTOTEST.java
>>>>>>> c68f9a4510d593887e8b1ebf4e1f4da5845d07fa:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autonomous/TensorflowObjectTest_NEEDTOTEST.java
=======
>>>>>>> c68f9a4510d593887e8b1ebf4e1f4da5845d07fa:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autonomous/TensorflowObjectTest.java

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AUoddv7/////AAABmdWoNVXEg09hhZk3YZfGMFsTIhkh2VwDTgEzX3nY+SV3FEUMCG4pHDYTDBEWUdsa5GcOyqevJQaXXHHHuK+8bmSKuV/WcO1oY5KIkFWC0IeN6RSmDYhCoadHTDLnrWZVb4b374j02ABfwSJgZyiuyWUrr0tbbnX/rOaxBoZ1/TyPsV8y0staXPbJKftaWVs1sxdvatjEI4DbiiFaTc5Amjlff+gUrr8GcWL/iuWi2azs8Wpr1KU7VtmazWx5nKW1qrnQFmcSoorGN2PxTYdEPJ4Y5dfx5PWcBMk47/1cH8+nCdbrbIGl5Om+QmKgKy3OabwWz4O5jJuC4g+bEMT4UR2JTvblEP9RFjf1KSo4TdS2";
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        FtcDashboard.getInstance().startCameraStream(webcam, 24);

        initVuforia();
        initTfod();


        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData("Test", "HELLO");
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            telemetry.update();

                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");



        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autonomous/TensorflowObjectTest_NEEDTOTEST.java
<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autonomous/TensorflowObjectTest.java
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        tfod.loadModelFromFile (TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT, LABEL_THIRD_ELEMENT);
=======
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
>>>>>>> c68f9a4510d593887e8b1ebf4e1f4da5845d07fa:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autonomous/TensorflowObjectTest_NEEDTOTEST.java
=======
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
>>>>>>> c68f9a4510d593887e8b1ebf4e1f4da5845d07fa:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autonomous/TensorflowObjectTest.java
    }
}