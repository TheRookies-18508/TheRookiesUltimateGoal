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
import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMapTheRookies;



@TeleOp
public class MecanumWheelTeleopStudio extends HardwareMapTheRookies{

    //motor 0 = bottomright
    //motor 1 = bottomleft
    //motor 2 = topleft
    //motor 3 = topright
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

;

    public double vertical;
    public double horiz1;
    public double horiz2;
    public double horiz3;
    public double horiz4;
    public double pivot;

    public double servoInitPos = 1;
    public double servoEndPos = 0;
    public static final double NEW_P = 1.17;
    public static final double NEW_I = 0.117;
    public static final double NEW_D = 0;
    public static final double NEW_F = 11.7;




    public void pullup(){
        servoArm.setPosition(.7);


    }
    public void setDown(){
        servoArm.setPosition(0);

    }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        init(hardwareMap);


        /*
        /makes the intake the front of the robot
        topleft.setDirection(DcMotor.Direction.FORWARD);
        topright.setDirection(DcMotor.Direction.REVERSE);
        bottomleft.setDirection(DcMotor.Direction.FORWARD);
        bottomright.setDirection(DcMotor.Direction.REVERSE);
         */






//        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




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

            topright.setPower((-pivot + (vertical - (horiz1 * .97))));
            bottomright.setPower((-pivot + vertical + (horiz1 * 1.3)) / 2);
            topleft.setPower((pivot + vertical + (horiz1 * 1.55)));
            bottomleft.setPower((pivot + (vertical - (horiz1 * 1.3))) / 2);
            PID_Shooter_Acticvate();
            //intake
            if (gamepad1.right_trigger > .2) {
                intakeOn();
            }

            if (gamepad1.dpad_up) {
                intake.setPower(-1);
                // intake.setVelocity(100);
                intake2.setPower(1);
                intake3.setPower(-.4);
            }
            if (gamepad1.left_trigger > .2) {
                intakeOff();

            }
            //gamepad 2
            if (gamepad2.right_trigger > .2) {
                intakeOn();
            }

            if (gamepad2.dpad_up) {
                intake.setPower(-1);
                // intake.setVelocity(100);
                intake2.setPower(1);
                intake3.setPower(-.4);
            }
            if (gamepad2.left_trigger > .2) {
                intakeOff();
            }


            //shart shooter wheel
            double initVelocity = 1700;
            double motorVelocity = 1750;
            double powershotVelocity = 1520;

            if (gamepad1.right_bumper) {

                shooterOn(towerVelo);
                intakeOff();

            } else if (gamepad1.left_bumper) {
                shooterOff();

            }

            if (gamepad1.dpad_right) {
                shooterOn(powerVelo);


            }
            if (gamepad1.b) {
                pullup();
            } else if (gamepad1.x) {
                setDown();
            } else if (gamepad1.dpad_left) {
                armServo.setPosition(.6);
            } else if (gamepad1.dpad_down) {
                armServo.setPosition(1);
            }

            //manual shooter
            if (FLY_WHEEL == true) {
                if (gamepad1.a) {
                    shootOne();
                    sleep(100);
                }

            }

            //auto shooter stuff
            if (FLY_WHEEL == true) {
                if (gamepad1.y) {
                    stopMotors();
                    sleep(300);
                    for (int i = 0; i <= 2; i++) {
                        shootOne();
                        sleep(400);
                    }
                    shooterOff();
                }

            }
            //gamepad 2 values
            if (gamepad2.right_bumper) {
                shooterOn(towerVelo);
                intakeOff();

            } else if (gamepad2.left_bumper) {
                shooterOff();

            }

            if (gamepad2.dpad_right) {
                shooterOn(powerVelo);

            }

            if (gamepad2.b) {
                pullup();
            } else if (gamepad2.x) {
                setDown();
            } else if (gamepad2.dpad_left) {
                armServo.setPosition(.6);
            } else if (gamepad2.dpad_down) {
                armServo.setPosition(1);
            }

            //manual shooter
            if (FLY_WHEEL == true) {
                if (gamepad2.a) {
                    shootOne();
                    sleep(300);
                }

            }

            //auto shooter stuff
            if (FLY_WHEEL == true) {
                if (gamepad2.y) {
                    sleep(300);
                    for (int i = 0; i <= 2; i++) {
                        shootOne();
                        sleep(400);
                    }
                    sleep(500);
                    shooterOff();

                }

            }


            if (shooter.getVelocity() == initVelocity) {
                telemetry.addData("Shooter Power: ", "Tower");
            } else if (shooter.getVelocity() == powershotVelocity) {
                telemetry.addData("Shooter Power: ", "Power Shots");
            } else if (shooter.getVelocity() == 0) {
                telemetry.addData("Shooter Power: ", 0);
            }
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Fly Wheel Status", FLY_WHEEL);
            telemetry.addData("Intake Status", INTAKE);
            telemetry.addData("Shooter Servo Position", servo1.getPosition());
            telemetry.addData("Runtime", "%.03f", getRuntime());


            telemetry.update();

            if (gamepad1.a) {
                shootOne();
            }

            if (gamepad1.y) {
                for (int i = 0; i <= 3; i++) {
                    shootOne();
                    sleep(400);
                }
            }


        }
    }

}