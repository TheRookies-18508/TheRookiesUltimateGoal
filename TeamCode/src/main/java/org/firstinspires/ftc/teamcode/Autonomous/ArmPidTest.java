package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.controller.PID;
import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMapTheRookies;

@Config
@Autonomous
public class ArmPidTest extends HardwareMapTheRookies {

    //public DcMotor arm;

    // delay in system that we intentionally add or is a known delay
    private double delay_ms = 20;
    public static double setPoint = 1000; // position we want our encoder to drive to
    private double initPoint = 0; //position we want oue encoder to start at

    // pid controller
    private PID PID;

    // pid gains
    public static final double kp = 0.006;
    public static final double ki = 0.000003;
    public static final double kd = 0.000005;


    @Override
    public void runOpMode() {

        //arm = hardwareMap.get(DcMotor.class, "arm");
//        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        arm.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        FtcDashboard dashboard;
        PID = new PID(kp,ki,kd,delay_ms);

        init(hardwareMap);

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {

            double current_state1 = topleft.getCurrentPosition();
            double current_state2 = topright.getCurrentPosition();
            double current_state3 = bottomleft.getCurrentPosition();
            double current_state4 = bottomright.getCurrentPosition();

            double plantOutput1 = PID.output(setPoint,current_state1);
            double plantOutput2 = PID.output(setPoint,current_state2);
            double plantOutput3 = PID.output(setPoint,current_state3);
            double plantOutput4 = PID.output(setPoint,current_state4);

            moveForwardEncoder(.2,3);

            customSleep(delay_ms);

            telemetry.addData("state: ", current_state1);
            telemetry.addData("setpoint: ", setPoint);
            telemetry.addData("error in system: ", setPoint - current_state1);
            telemetry.addData("plant output: ", plantOutput1);
            telemetry.addData("kp: ",kp);
            telemetry.addData("ki: ",ki);
            telemetry.addData("kd",kd);
            telemetry.update();

            break;

        }


    }




}