//Teleop program for TheRookies for the Ultimate Goal Season
package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;


@TeleOp
public class MecanumWheelTeleopStudio extends LinearOpMode{

    //motor 0 = bottomright
    //motor 1 = bottomleft
    //motor 2 = topleft
    //motor 3 = topright
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor topleft = null;
    DcMotor topright = null;
    DcMotor bottomleft = null;
    DcMotor bottomright = null;
    DcMotorEx intake = null;
    DcMotor intake2 = null;
    DcMotor intake3 = null;
    Servo testservo = null;
    public Servo servo1 = null;
    public Servo servoArm = null;
    public DcMotorEx shooter = null;

    public double vertical;
    public double horiz1;
    public double horiz2;
    public double horiz3;
    public double horiz4;
    public double pivot;
    public boolean FLY_WHEEL = false;
    public boolean INTAKE = false;
    public double INTAKE_SPEED = .8;
    public double servoInitPos = 1;
    public double servoEndPos = 0;
    public static final double NEW_P = 1.17;
    public static final double NEW_I = 0.117;
    public static final double NEW_D = 0;
    public static final double NEW_F = 11.7;



    public void shoot(){
        servo1.setPosition(1);
        servo1.setPosition(0);
        sleep(350);
        servo1.setPosition(1);
    }
    public void pullup(){
        testservo.setPosition(.6);
        servoArm.setPosition(.8);
    }
    public void setDown(){
        servoArm.setPosition(0);
        testservo.setPosition(1);
    }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        bottomright  = hardwareMap.get(DcMotor.class, "bottomright");
        bottomleft = hardwareMap.get(DcMotor.class, "bottomleft");
        topleft = hardwareMap.get(DcMotor.class, "topleft");
        topright = hardwareMap.get(DcMotor.class, "topright");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake3 = hardwareMap.get(DcMotorEx.class, "intake3");
        servo1 = hardwareMap.servo.get("Servo1");
        servoArm = hardwareMap.servo.get("servoArm");
        testservo = hardwareMap.servo.get("testservo");

        /*
        /makes the intake the front of the robot
        topleft.setDirection(DcMotor.Direction.FORWARD);
        topright.setDirection(DcMotor.Direction.REVERSE);
        bottomleft.setDirection(DcMotor.Direction.FORWARD);
        bottomright.setDirection(DcMotor.Direction.REVERSE);
         */

        topleft.setDirection(DcMotor.Direction.REVERSE);
        topright.setDirection(DcMotor.Direction.FORWARD);
        bottomleft.setDirection(DcMotor.Direction.REVERSE);
        bottomright.setDirection(DcMotor.Direction.FORWARD);
        intake3.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotor.Direction.FORWARD);
        intake2.setDirection(DcMotor.Direction.FORWARD);


        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        topleft = hardwareMap.dcMotor.get("topleft");
        topright = hardwareMap.dcMotor.get("topright");
        bottomleft = hardwareMap.dcMotor.get("bottomleft");
        bottomright = hardwareMap.dcMotor.get("bottomright");


        //MainArm.setPosition(MainArm_Home); // sets to starting position
        //SideArm.setPosition(.85);
        FLY_WHEEL = false;
        INTAKE = false;


       




        waitForStart();
        servo1.setPosition(1);
        while (opModeIsActive()) {


            double rightstickY = -gamepad1.right_stick_y;
            double leftstickY = -gamepad1.left_stick_y;
            double rightstickX = -gamepad1.right_stick_x;
            double leftstickX = -gamepad1.left_stick_x;
            double twoleftstickY = gamepad2.left_stick_y;
            double twoleftstickX = -gamepad2.left_stick_x;
            boolean rightbumper = gamepad1.right_bumper;
            boolean leftbumper = gamepad1.left_bumper;


            vertical = -gamepad1.left_stick_y;
            horiz1 = gamepad1.right_stick_x; //strafing
            pivot = (-gamepad1.left_stick_x);

            topright.setPower((-pivot + (vertical - (horiz1*.97))));
            bottomright.setPower((-pivot + vertical + (horiz1 * 1.3))/2);
            topleft.setPower((pivot + vertical + (horiz1 * 1.55)));
            bottomleft.setPower((pivot + (vertical - (horiz1 * 1.3)))/2);



            //intake
            if (gamepad1.right_trigger > .2){
                intake.setPower(1);
                // intake.setVelocity(100);
                intake2.setPower(-1);
                intake3.setPower(1);
                shooter.setVelocity(-400);
                INTAKE = true;
                //INTAKE_SPEED = .6;
            }

            if (gamepad1.dpad_up){
                intake.setPower(-1);
                // intake.setVelocity(100);
                intake2.setPower(1);
                intake3.setPower(-.4);
            }
            if (gamepad1.left_trigger > .2){
                intake.setPower(0);
                intake2.setPower(0);
                intake3.setPower(0);
                shooter.setVelocity(0);
                INTAKE = false;
                INTAKE_SPEED = .8;
                FLY_WHEEL = false;
            }




            //shart shooter wheel
            double initVelocity = 1700;
            double motorVelocity = 1750;
            double powershotVelocity = 1520;

            if (gamepad1.right_bumper) {
                motorVelocity = initVelocity;
                shooter.setVelocity(motorVelocity);
                FLY_WHEEL = true;
                INTAKE_SPEED = .6;
                intake.setPower(0);
                intake2.setPower(0);
                intake3.setPower(0);
                INTAKE = false;

            } else if (gamepad1.left_bumper) {
                shooter.setPower(0);
                FLY_WHEEL = false;
                INTAKE_SPEED = .8;
            }

            if (gamepad1.dpad_right){
                motorVelocity = powershotVelocity;
                shooter.setVelocity(motorVelocity);
                FLY_WHEEL = true;

            }
            if (gamepad1.b){
                pullup();
            }
            else if (gamepad1.x){
                setDown();
            }

            //manual shooter
            if (FLY_WHEEL == true) {
                if (gamepad1.a) {
                    shoot();
                    sleep(300);
                }

            }

            //auto shooter stuff
            if (FLY_WHEEL == true) {
                if (gamepad1.y) {
                    sleep(300);
                    for (int i = 0; i <= 2; i++) {
                        shoot();
                        sleep(400);
                    }
                    sleep(500);
                    shooter.setPower(0);
                    FLY_WHEEL = false;
                }

            }



            if (shooter.getVelocity() == initVelocity){
                telemetry.addData("Shooter Power: ", "Tower");
            }
            else if(shooter.getVelocity() == powershotVelocity){
                telemetry.addData("Shooter Power: ", "Power Shots");
            }
            else if(shooter.getVelocity() == 0){
                telemetry.addData("Shooter Power: ", 0);
            }
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Fly Wheel Status", FLY_WHEEL);
            telemetry.addData("Intake Status", INTAKE);
            telemetry.addData("Shooter Servo Position", servo1.getPosition());
            telemetry.addData("Runtime", "%.03f", getRuntime());



            telemetry.update();

        }

    }
}