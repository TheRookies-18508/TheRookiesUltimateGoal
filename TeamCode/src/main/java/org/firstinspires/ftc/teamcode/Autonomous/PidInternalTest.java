package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMapTheRookies;

@Autonomous
public class PidInternalTest extends LinearOpMode {

    // our DC motor.
    public DcMotor topleft = null;
    public DcMotor topright = null;
    public DcMotor bottomleft = null;
    public DcMotor bottomright = null;


    public static final double NEW_P = 1.17;
    public static final double NEW_I = 0.117;
    public static final double NEW_D = 0;
    public static final double NEW_F = 11.7;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1000;
    static final double DRIVE_GEAR_REDUCTION_TOP = 1.0;     // This is < 1.0 if geared UP
    static final double DRIVE_GEAR_REDUCTION_BOTTOM = .5;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.54331;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_INCH_BOTTOM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION_BOTTOM) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public void runOpMode() {
        // get reference to DC motor.
        topleft = hardwareMap.get(DcMotor.class, "topleft");
        topright = hardwareMap.get(DcMotor.class, "topright");
        bottomleft = hardwareMap.get(DcMotor.class, "bottomleft");
        bottomright = hardwareMap.get(DcMotor.class, "bottomright");

        topleft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        topright.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        bottomleft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        bottomright.setDirection(DcMotor.Direction.FORWARD);
        
        // wait for start command.
        waitForStart();

        // get a reference to the motor controller and cast it as an extended functionality controller.
        // we assume it's a REV Robotics Expansion Hub (which supports the extended controller functions).
        DcMotorControllerEx motorControllertl = (DcMotorControllerEx)topleft.getController();
        DcMotorControllerEx motorControllertr = (DcMotorControllerEx)topright.getController();
        DcMotorControllerEx motorControllerbl = (DcMotorControllerEx)bottomleft.getController();
        DcMotorControllerEx motorControllerbr = (DcMotorControllerEx)bottomright.getController();

        // get the port number of our configured motor.
        int motorIndextl = ((DcMotorEx)topleft).getPortNumber();
        int motorIndextr = ((DcMotorEx)topright).getPortNumber();
        int motorIndexbr = ((DcMotorEx)bottomright).getPortNumber();
        int motorIndexbl = ((DcMotorEx)bottomleft).getPortNumber();

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDFCoefficients pidOrigtl = motorControllertl.getPIDFCoefficients(motorIndextl, DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidOrigtr = motorControllertr.getPIDFCoefficients(motorIndextr, DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidOrigbr = motorControllerbl.getPIDFCoefficients(motorIndexbr, DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidOrigbl = motorControllerbr.getPIDFCoefficients(motorIndexbl, DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients.
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        motorControllertl.setPIDFCoefficients(motorIndextl, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        motorControllertr.setPIDFCoefficients(motorIndextr, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        motorControllerbl.setPIDFCoefficients(motorIndexbr, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        motorControllerbr.setPIDFCoefficients(motorIndexbl, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        // re-read coefficients and verify change.
        PIDFCoefficients pidModifiedtl = motorControllertl.getPIDFCoefficients(motorIndextl, DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidModifiedtr = motorControllertr.getPIDFCoefficients(motorIndextr, DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidModifiedbl = motorControllerbl.getPIDFCoefficients(motorIndexbr, DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidModifiedbr = motorControllerbr.getPIDFCoefficients(motorIndexbl, DcMotor.RunMode.RUN_USING_ENCODER);

        // display info to user.
        while(opModeIsActive()) {

            moveForwardEncoder(.2,10);


            telemetry.addData("Runtime", "%.03f", getRuntime());
            telemetry.addData("P,I,D,F (orig)", "%.04f, %.04f, %.0f, %.0f",
                    pidOrigtr.p, pidOrigtr.i, pidOrigtr.d, pidOrigtr.f);
            telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f, %.04f",
                    pidModifiedtr.p, pidModifiedtr.i, pidModifiedtr.d, pidModifiedtr.f);
            telemetry.update();
        }
    }
    public void moveForwardEncoder(double speed, double inches){
        topleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderDrive(speed, inches, inches, inches, inches, 100);
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

                telemetry.addData("Path1",  "Running to %7d :%7d:%7d:%7d", newtopLeftTarget,  newtopRightTarget, newbottomLeftTarget, newbottomRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d:%7d:%7d",
                        topleft.getCurrentPosition(),
                        bottomleft.getCurrentPosition(),
                        topright.getCurrentPosition(),
                        bottomright.getCurrentPosition());
                telemetry.update();
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

}