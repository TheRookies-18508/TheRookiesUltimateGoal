
package org.firstinspires.ftc.teamcode.HardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.controller.PID;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode.
 <<<<<<< HEAD
 <<<<<<< HEAD

 *
 * This hardware class assumes the following device names have been configured on the robot:
 * CHAWKS:  Naming convention is camel case!
 *
 *          front
 *    (LF)--------(RF)
 *    |    robot   |
 *   (LB)--------(RB)
 *        back
 *
 =======
 =======
 >>>>>>> origin/r2_park
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * CHAWKS:  Naming convention is camel case!
 * <p>
 * front
 * (LF)--------(RF)
 * |    robot   |
 * (LB)--------(RB)
 * back
 * <p>
 <<<<<<< HEAD
 >>>>>>> origin/r1_park
 =======
 >>>>>>> origin/r2_park
 * Motor channel:  Left Front (LF) drive motor:        "topleft"
 * Motor channel:  Right Front (RF) drive motor:        "topright"
 * Motor channel:  Left Back (LB) drive motor:        "bottomleft"
 * Motor channel:  Right Back (RB) drive motor:        "bottomright"
 */

public abstract class HardwareMapTheRookies extends LinearOpMode
{

    /* Public OpMode members. */
    // CHAWKS: The Robot Parts need to be established here
    public DcMotor topleft = null;
    public DcMotor topright = null;
    public DcMotor bottomleft = null;
    public DcMotor bottomright = null;
    public DcMotorEx shooter = null;

    public double motorVelocity = 1790;
    public double tl;
    public double tr;
    public double bl;
    public double br;

    public Servo servo1 = null;
    public Servo armServo = null;
    public Servo servoArm = null;
    static final double THRESHOLD = 1.5;
    BNO055IMU imu;

    private double delay_ms = 20;
    private double initPoint = 0; //position we want oue encoder to start at

    // pid controller
    private PID PID;

    // pid gains
    private final double kp = 0.006;
    private final double ki = 0.000003;
    private final double kd = 0.000005;

    /////////////////////////////////////

    /* local OpMode members. */
    com.qualcomm.robotcore.hardware.HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    /* CHAWKS: Call and declare the robot here */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor topleft1;

    /* Constructor */
    public HardwareMapTheRookies() {

    }

    static final double COUNTS_PER_MOTOR_REV = 1000;
    static final double DRIVE_GEAR_REDUCTION_TOP = 1.0;     // This is < 1.0 if geared UP
    static final double DRIVE_GEAR_REDUCTION_BOTTOM = .5;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.54331;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_INCH_BOTTOM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION_BOTTOM) / (WHEEL_DIAMETER_INCHES * 3.1415);
  

    // Initialize standard Hardware interfaces
    /*
     */
    public void init(com.qualcomm.robotcore.hardware.HardwareMap ahwMap) {
        // Save reference to Hardware map
        // Do Not ever remove this.
        hwMap = ahwMap;

        // Define and Initialize Motors
        /*
            CHAWKS: The deviceName should ALWAYS ALWAYS ALWAYS
                    match the part name to avoid confusion
         */
        topleft = hwMap.get(DcMotor.class, "topleft");
        topright = hwMap.get(DcMotor.class, "topright");
        bottomleft = hwMap.get(DcMotor.class, "bottomleft");
        bottomright = hwMap.get(DcMotor.class, "bottomright");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        servo1 = hardwareMap.servo.get("Servo1");
        armServo = hardwareMap.servo.get("testservo");
        servoArm = hardwareMap.servo.get("servoArm");


//Threshold for Gyro turning so that we will not continuously attempt to reach an exact value


        // Set Direction/Motion for Motors
        /*
         */
        topleft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        topright.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        bottomleft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        bottomright.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        PID = new PID(kp,ki,kd,delay_ms);


        // Set all motors to ZERO! power
        /*
            CHAWKS: Why do we set the power to zero?
         */
        topleft.setPower(0);
        topright.setPower(0);
        bottomleft.setPower(0);
        bottomright.setPower(0);

        // Set all motors to run without encoders.
        /*
            CHAWKS: Encoder Exercise!
         */

        //bottomleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //bottomright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        //get and initialize IMU
        imu = hwMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        servoArm.setPosition(0);
        sleep(2000);
        servo1.setPosition(1);
    }

    /*
     *  CHAWKS: It's a METHOD!!!
     *
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */


    public void dropWobble_park(){
//        double setPoint = 4; // position we want our encoder to drive to
//        double current_state = arm.getCurrentPosition();
//        double plantOutput = PID.output(setPoint,current_state);
//        arm.setPower(plantOutput);
//
//        customSleep(delay_ms);
//
//        telemetry.addData("state: ", current_state);
//        telemetry.addData("setpoint: ", setPoint);
//        telemetry.addData("error in system: ", setPoint - current_state);
//        telemetry.addData("plant output: ", plantOutput);


//        arm.setPower(.35);
        servoArm.setPosition(.7);
        sleep(500);
        hold();
        sleep(1000);

        strafeLeft(.4,15);
//        arm.setPower(-.3);
        servoArm.setPosition(0);
        sleep(1000);
        moveBackEncoder(.6, 29);
    }

    public void shoot(int all){
        //shooter.setVelocity(motorVelocity);
        sleep(600);
        servo1.setPosition(1);
        servo1.setPosition(0);
        sleep(350);
        servo1.setPosition(1);
        if (all != 1) {
            sleep(750);
            shooter.setVelocity(0);
        }
        else{
            shooter.setVelocity(motorVelocity);
        }
    }

    public void shootOne(){
        servo1.setPosition(.85);
        servo1.setPosition(0);
        sleep(350);
        servo1.setPosition(.85);
    }
    public void shootAll(){
        for (int i = 0; i <= 2; i++) {
            shoot(1);
            sleep(700);
        }
        sleep(200);
    }

    public void shootTop(){
        moveForwardEncoder(.3,59);
        sleep(300);
        gyroTurn(.5,-3.7);
        shooter.setVelocity(motorVelocity);
        sleep(1000);
        shootAll();
        sleep(200);
        shooter.setVelocity(0);
        gyroTurn(.5,3.7);
        sleep(200);
        moveForwardEncoder(.5,16);

    }


    public void hold(){
        armServo.setPosition(1);
    }

    public void leggo(){
        armServo.setPosition(0.6);
    }

    public void gyroTurn(double power, double target) {
        Orientation angles;
        double error;
        double k = 6 / 360.0;
        double kInt = 3 / 3600.0;
        double eInt = 0;
        double prevTime = System.currentTimeMillis();
        double globalAngle = 0;
        double lastAngle = 0;
        double deltaAngle = 0;
        while (opModeIsActive()) {
            double currentTime = System.currentTimeMillis();
            double loopTime = (currentTime - prevTime) / 1000.0; // In seconds
            prevTime = currentTime;
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //finds the angle given by the imu [-180, 180]
            double angle = angles.firstAngle;
            deltaAngle = angle - lastAngle;

            //adjusts the change in angle (deltaAngle) to be the actual change in angle
            if (deltaAngle < -180) {
                deltaAngle += 360;
            } else if (deltaAngle > 180) {
                deltaAngle -= 360;
            }
            globalAngle += deltaAngle;
            lastAngle = angle;

            error = (target - globalAngle)*-1;
            eInt += loopTime * error;
            telemetry.addData("Heading", angles.firstAngle + " degrees");
            telemetry.addData("GlobalAngle", globalAngle + " degrees");
            telemetry.addData("Error", error + " degrees");
            telemetry.addData("Loop time: ", loopTime + " ms");
            telemetry.update();

//            if (error > -1.5 && error < 1.5) {
//                stopMotors();
//                telemetry.addData("INside","loop");
//                break;
//            }
//            if (power > 0){
//                turnRight(Math.abs(((k * error + kInt * eInt))));
//            }
//            else if(power < 0) {
//                turnLeft((k * error + kInt * eInt) * -1);
//            }

            if (error > -1.5 && error < 1.5) {
                stopMotors();
                break;
            }
            turnLeft((k * error + kInt * eInt) );
            idle();

        }
    }

    public void customSleep(double delay) {
        long startOfDelayTime = System.currentTimeMillis();

        while ((startOfDelayTime + delay > System.currentTimeMillis()) && opModeIsActive()) {

        }


    }


    public void goForward(double power, int distance) {
        Orientation angles;
        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double error;
        double k = .025;
        int topleftTarget = topleft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int toprightTarget = topright.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int bottomleftTarget = bottomleft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH_BOTTOM);
        int bottomrightTarget = bottomright.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH_BOTTOM);

        while (opModeIsActive() &&
                (power > 0 && topleft.getCurrentPosition() < topleftTarget && topright.getCurrentPosition() < toprightTarget && bottomleft.getCurrentPosition() < bottomleftTarget && bottomright.getCurrentPosition() < bottomrightTarget) ||
                (power < 0 && topleft.getCurrentPosition() > topleftTarget && topright.getCurrentPosition() > toprightTarget && bottomleft.getCurrentPosition() > bottomleftTarget && bottomright.getCurrentPosition() > bottomrightTarget)
        ) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //finds the angle given by the imu [-180, 180]
            double angle = angles.firstAngle;
            error = startAngle - angle;
            telemetry.addData("firstAngle",angles.firstAngle+" degrees");
            telemetry.addData("topleft ",topleft.getCurrentPosition());
            telemetry.addData("topright ",topright.getCurrentPosition());
            telemetry.addData("bottomleft ",bottomleft.getCurrentPosition());
            telemetry.addData("bottomright ",bottomright.getCurrentPosition());

            telemetry.update();
            topleft.setPower((power - (error * k)));
            topright.setPower((power + (error * k)));
            bottomleft.setPower((power - (error * k)));
            bottomright.setPower((power + (error * k)));
        }
        stopMotors();

    }

    public void goForward2(double power, int distance) {
        Orientation angles;
        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double error;
        double k = 3/360.0;
        int topleftTarget = topleft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int toprightTarget = topright.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int bottomleftTarget = bottomleft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int bottomrightTarget = bottomright.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);

        while (opModeIsActive() &&
                (power > 0 && topleft.getCurrentPosition() < topleftTarget && topright.getCurrentPosition() < toprightTarget && bottomleft.getCurrentPosition() < bottomleftTarget && bottomright.getCurrentPosition() < bottomrightTarget) ||
                (power < 0 && topleft.getCurrentPosition() > topleftTarget && topright.getCurrentPosition() > toprightTarget && bottomleft.getCurrentPosition() > bottomleftTarget && bottomright.getCurrentPosition() > bottomrightTarget)
        ) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //finds the angle given by the imu [-180, 180]
            double angle = angles.firstAngle;
            error = startAngle - angle;
            telemetry.addData("firstAngle",angles.firstAngle+" degrees");
            telemetry.addData("topleft ",topleft.getCurrentPosition());
            telemetry.addData("topright ",topright.getCurrentPosition());
            telemetry.addData("bottomleft ",bottomleft.getCurrentPosition());
            telemetry.addData("bottomright ",bottomright.getCurrentPosition());

            telemetry.update();
            topleft.setPower((power - (error * k)));
            topright.setPower((power + (error * k)));
            bottomleft.setPower((power - (error * k)));
            bottomright.setPower((power + (error * k)));
        }
        stopMotors();

    }

    public void goBackward(double power, int distance) {
        goForward(-power, -distance);
    }

    public void turnLeft(double power) {
        topleft.setPower(-power);
        topright.setPower(power);
        bottomleft.setPower(-power);
        bottomright.setPower(power);
    }

    public void turnRight(double power) {
        topleft.setPower(power);
        topright.setPower(-power);
        bottomleft.setPower(power);
        bottomright.setPower(-power);
    }
    
    public void strafeRight(double power, int distance) {
        Orientation angles;
        double error;
        double k = .09;
        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        int topleftTarget = topleft.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        int toprightTarget = topright.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
        int bottomleftTarget = bottomleft.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
        int bottomrightTarget = bottomright.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        topleft.setTargetPosition(topleftTarget);
        topright.setTargetPosition(toprightTarget);
        bottomleft.setTargetPosition(bottomleftTarget);
        bottomright.setTargetPosition(bottomrightTarget);

        while (opModeIsActive()
                && (topleft.getCurrentPosition() < topleftTarget && topright.getCurrentPosition() > toprightTarget && bottomleft.getCurrentPosition() > bottomleftTarget && bottomright.getCurrentPosition() < bottomrightTarget)) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //finds the angle given by the imu [-180, 180]
            double angle = angles.firstAngle;
            error = startAngle - angle;
            topleft.setPower((power - (error * k))+.03);
            topright.setPower(-(power - (error * k))+.03);
            bottomleft.setPower(-(power + (error * k)));
            bottomright.setPower((power + (error * k)));
            telemetry.addData("error: ", error);

//            telemetry.addData("topleft dest: ", topleftTarget);
//            telemetry.addData("topleft pos: ", topleft.getCurrentPosition());
//
//            telemetry.addData("topright dest: ", toprightTarget);
//            telemetry.addData("topright pos: ", topright.getCurrentPosition());
//
//            telemetry.addData("bottomleft dest: ", bottomleftTarget);
//            telemetry.addData("bottomleft pos: ", bottomleft.getCurrentPosition());
//
//
//            telemetry.addData("bottomright dest: ", bottomrightTarget);
//            telemetry.addData("bottomright pos: ", bottomright.getCurrentPosition());

            telemetry.update();


            telemetry.update();

        }
        stopMotors();
    }

    public void strafeLeft(double power, double distance) {
        Orientation angles;
        double error;
        double k = .09;
        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        int topleftTarget = topleft.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
        int toprightTarget = topright.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        int bottomleftTarget = bottomleft.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH_BOTTOM);
        int bottomrightTarget = bottomright.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH_BOTTOM);
        topleft.setTargetPosition(topleftTarget);
        topright.setTargetPosition(toprightTarget);
        bottomleft.setTargetPosition(bottomleftTarget);
        bottomright.setTargetPosition(bottomrightTarget);

        while (opModeIsActive()
                && (topleft.getCurrentPosition() > topleftTarget && topright.getCurrentPosition() < toprightTarget && bottomleft.getCurrentPosition() < bottomleftTarget && bottomright.getCurrentPosition() > bottomrightTarget)) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //finds the angle given by the imu [-180, 180]
            double angle = angles.firstAngle;
            error = (startAngle - angle)*-1;
            topleft.setPower(-(power + (error * k))+.03);
            topright.setPower((power + (error * k))+.03);
            bottomleft.setPower((power - (error * k)));
            bottomright.setPower(-(power - (error * k)));
//            telemetry.addData("error: ", error);
//
//            telemetry.addData("topleft dest: ", topleftTarget);
//            telemetry.addData("topleft pos: ", topleft.getCurrentPosition());
//
//            telemetry.addData("topright dest: ", toprightTarget);
//            telemetry.addData("topright pos: ", topright.getCurrentPosition());
//
//            telemetry.addData("bottomleft dest: ", bottomleftTarget);
//            telemetry.addData("bottomleft pos: ", bottomleft.getCurrentPosition());
//
//
//            telemetry.addData("bottomright dest: ", bottomrightTarget);
//            telemetry.addData("bottomright pos: ", bottomright.getCurrentPosition());
//
//            telemetry.update();

        }
        stopMotors();
    }
    public void turnLeftNew(double turnAngle, double timeoutS) {
        Orientation angles;
        sleep(500);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double speed = .5;
        double oldDegreesLeft = turnAngle;
        double scaledSpeed = speed;
        double targetHeading = angles.firstAngle + turnAngle;
        double oldAngle = angles.firstAngle;
        telemetry.addData("Target Heading", targetHeading);
        telemetry.addData("old angle", oldAngle);

        telemetry.update();
//        if (targetHeading < -180) {
//            targetHeading += 360;
//        }
//        if (targetHeading > 180) {
//            targetHeading -= 360;
//        }
        double degreesLeft = ((int)(Math.signum(angles.firstAngle - targetHeading) + 1) / 2) * (360 - Math.abs(angles.firstAngle - targetHeading)) + (int)(Math.signum(targetHeading - angles.firstAngle) + 1) / 2 * Math.abs(angles.firstAngle - targetHeading);
        runtime.reset();
        while (opModeIsActive() &&
                runtime.seconds() < timeoutS &&
                degreesLeft > 1 &&
                oldDegreesLeft - degreesLeft >= 0) { //check to see if we overshot target
            scaledSpeed = degreesLeft / (100 + degreesLeft) * speed;
            if (scaledSpeed > 1) {
                scaledSpeed = .1;
            }
            bottomleft.setPower(scaledSpeed * 1.3); //extra power to back wheels
            bottomright.setPower(-1 * scaledSpeed * 1.3); //due to extra weight
            topleft.setPower(scaledSpeed);
            topright.setPower(-1 * scaledSpeed);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            oldDegreesLeft = degreesLeft;
            degreesLeft = ((int)(Math.signum(angles.firstAngle - targetHeading) + 1) / 2) * (360 - Math.abs(angles.firstAngle - targetHeading)) + (int)(Math.signum(targetHeading - angles.firstAngle) + 1) / 2 * Math.abs(angles.firstAngle - targetHeading);
            if (Math.abs(angles.firstAngle - oldAngle) < 1) {
                speed *= 1.1;
            } //bump up speed to wheels in case robot stalls before reaching target
            oldAngle = angles.firstAngle;
        }
        stopMotors(); //our helper method to set all wheel motors to zero
        sleep(250); //small pause at end of turn
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
            newtopLeftTarget = topleft.getCurrentPosition() + (int)(topleftInches * COUNTS_PER_INCH);
            newtopRightTarget = topright.getCurrentPosition() + (int)(toprightInches * COUNTS_PER_INCH);
            newbottomLeftTarget = bottomleft.getCurrentPosition() + (int)(bottomleftInches * COUNTS_PER_INCH_BOTTOM);
            newbottomRightTarget = bottomright.getCurrentPosition() + (int)(bottomrightInches * COUNTS_PER_INCH_BOTTOM);
            topleft.setTargetPosition(newtopLeftTarget);
            topright.setTargetPosition(newtopRightTarget);
            bottomleft.setTargetPosition(newbottomLeftTarget);
            bottomright.setTargetPosition(newbottomRightTarget);

            // Turn On RUN_TO_POSITION
            topleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            topright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bottomleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bottomright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            topleft.setPower((speed));
            topright.setPower((speed));
            bottomleft.setPower((speed));
            bottomright.setPower((speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (topleft.isBusy() && topright.isBusy() && bottomleft.isBusy() && bottomright.isBusy())) {

//                telemetry.addData("Path1",  "Running to %7d :%7d:%7d:%7d", newtopLeftTarget,  newtopRightTarget, newbottomLeftTarget, newbottomRightTarget);
//                telemetry.addData("Path2",  "Running at %7d :%7d:%7d:%7d",
//                        topleft.getCurrentPosition(),
//                        bottomleft.getCurrentPosition(),
//                        topright.getCurrentPosition(),
//                        bottomright.getCurrentPosition());
//                telemetry.update();
            }

            // Stop all motion;
            topleft.setPower(0);
            topright.setPower(0);
            bottomleft.setPower(0);
            bottomright.setPower(0);

            // Turn off RUN_TO_POSITION
            topleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            topright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottomleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottomright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }





    //without IMU
    public void moveForwardEncoder(double speed, double inches){
        topleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderDrive(speed, inches, inches, inches, inches, 100);
    }

    public void moveBackEncoder(double speed, double inches){
        topleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderDrive(speed, -inches, -inches, -inches , -inches, 100);
    }

    public void strafeRightEncoder(double speed, double inches){
        topleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderDrive(speed, inches*.97, -inches*1.3, -inches *1.55 ,inches *1.3, 100);
    }

    public void stopMotors() {
        topleft.setPower(0);
        bottomleft.setPower(0);
        topright.setPower(0);
        bottomright.setPower(0);
    }
}
