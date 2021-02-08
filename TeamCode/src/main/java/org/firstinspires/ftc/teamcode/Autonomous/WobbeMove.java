package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;


@Autonomous
public class WobbeMove extends LinearOpMode{

    private DcMotor topleftAsDcMotor;
    private DcMotor bottomleftAsDcMotor;
    private DcMotor toprightAsDcMotor;
    private DcMotor bottomrightAsDcMotor;
    public Servo servo1 = null;
    public DcMotorEx shooter = null;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 753.2;    // GoBILDA Motor
    static final double DRIVE_GEAR_REDUCTION_TOP = 1.0;     // This is < 1.0 if geared UP
    static final double DRIVE_GEAR_REDUCTION_BOTTOM = .5;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.54331;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_INCH_BOTTOM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION_BOTTOM) / (WHEEL_DIAMETER_INCHES * 3.1415);


    private VuforiaSkyStone vuforiaSkyStone;
    private double skystoneYpos;
    private VuforiaBase.TrackingResults vuforiaResults;

    @Override
    public void runOpMode() {
        vuforiaSkyStone = new VuforiaSkyStone();
        topleftAsDcMotor = hardwareMap.dcMotor.get("topleft");
        bottomleftAsDcMotor = hardwareMap.dcMotor.get("bottomleft");
        toprightAsDcMotor = hardwareMap.dcMotor.get("topright");
        bottomrightAsDcMotor = hardwareMap.dcMotor.get("bottomright");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        servo1 = hardwareMap.servo.get("Servo1");

        topleftAsDcMotor.setDirection(DcMotor.Direction.REVERSE);
        toprightAsDcMotor.setDirection(DcMotor.Direction.FORWARD);
        bottomleftAsDcMotor.setDirection(DcMotor.Direction.REVERSE);
        bottomrightAsDcMotor.setDirection(DcMotor.Direction.FORWARD);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double motorVelocity = 1700;


        servo1.setPosition(.75);
        waitForStart();
        while(opModeIsActive()){
            moveForward(.2,10);
            turn(.2, -2.5, 2.5);
            shooter.setVelocity(motorVelocity);
            sleep(1000);
            for (int i = 0; i <= 2; i++) {
                shoot();
                sleep(750);
            }
            sleep(500);
            moveForward(.6,75);
            shooter.setPower(0);
            break;
        }


    }
    public void moveForward(double speed, double inches){
        topleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        toprightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderDrive(speed, inches, inches, inches, inches, 100);
    }

    public void moveBack(double speed, double inches){
        topleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        toprightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderDrive(speed, -inches, -inches, -inches , -inches, 100);
    }

    public void strafeLeft(double speed, double inches){
        topleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        toprightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderDrive(speed, -inches, inches, inches ,-inches, 100);
    }

    public void strafeRight(double speed, double inches){
        topleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        toprightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderDrive(speed, inches, -inches, -inches ,inches, 100);
    }
    public void turn(double speed, double inches, double inches2){
        topleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        toprightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderDrive(speed, inches, inches2, inches ,inches2, 100);
    }
    public void shoot(){
        servo1.setPosition(.75);
        servo1.setPosition(0);
        sleep(350);
        servo1.setPosition(.75);
    }





    public void encoderDrive(double speed,
                             double topleftInches, double toprightInches, double bottomleftInches, double bottomrightInches,
                             double timeoutS) {
        int newtopLeftTarget;
        int newtopRightTarget;
        int newbottomLeftTarget;
        int newbottomRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newtopLeftTarget = topleftAsDcMotor.getCurrentPosition() + (int)(topleftInches * COUNTS_PER_INCH);
            newtopRightTarget = toprightAsDcMotor.getCurrentPosition() + (int)(toprightInches * COUNTS_PER_INCH);
            newbottomLeftTarget = bottomleftAsDcMotor.getCurrentPosition() + (int)(bottomleftInches * COUNTS_PER_INCH_BOTTOM);
            newbottomRightTarget = bottomrightAsDcMotor.getCurrentPosition() + (int)(bottomrightInches * COUNTS_PER_INCH_BOTTOM);
            topleftAsDcMotor.setTargetPosition(newtopLeftTarget);
            toprightAsDcMotor.setTargetPosition(newtopRightTarget);
            bottomleftAsDcMotor.setTargetPosition(newbottomLeftTarget);
            bottomrightAsDcMotor.setTargetPosition(newbottomRightTarget);

            // Turn On RUN_TO_POSITION
            topleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            toprightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bottomleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bottomrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            topleftAsDcMotor.setPower((speed));
            toprightAsDcMotor.setPower((speed));
            bottomleftAsDcMotor.setPower((speed));
            bottomrightAsDcMotor.setPower((speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (topleftAsDcMotor.isBusy() && toprightAsDcMotor.isBusy() && bottomleftAsDcMotor.isBusy() && bottomrightAsDcMotor.isBusy())) {

                telemetry.addData("Path1",  "Running to %7d :%7d:%7d:%7d", newtopLeftTarget,  newtopRightTarget, newbottomLeftTarget, newbottomRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d:%7d:%7d",
                        topleftAsDcMotor.getCurrentPosition(),
                        bottomleftAsDcMotor.getCurrentPosition(),
                        toprightAsDcMotor.getCurrentPosition(),
                        bottomrightAsDcMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            topleftAsDcMotor.setPower(0);
            toprightAsDcMotor.setPower(0);
            bottomleftAsDcMotor.setPower(0);
            bottomrightAsDcMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            topleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            toprightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottomleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottomrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
