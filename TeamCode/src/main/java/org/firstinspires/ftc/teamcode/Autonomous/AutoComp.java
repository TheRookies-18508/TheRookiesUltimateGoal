package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMapTheRookies;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

@Autonomous
public class AutoComp extends HardwareMapTheRookies{

    @Override
    public void runOpMode() {

        telemetry.addData("Status: ", "Hit [Init] to Initialize the bot");    //
        telemetry.update();



        init(hardwareMap);
        waitForStart();

        while(opModeIsActive()){
//hit top goal


           // hit 3 power shots
//            moveForwardEncoder(.1,28);
//            shooter.setVelocity(motorVelocity);
//            sleep(300);
//            strafeLeft(.2,7);
//            sleep(400);
//            gyroTurn(.2,-4);
//            shootOne();
//            strafeLeft(.2,8);
//            moveBackEncoder(.2,6);
//            sleep(400);
//            gyroTurn(.2,-3);
//            shootOne();
//            strafeLeft(.2,7);
//            sleep(400);
//            moveForwardEncoder(.2,3);
//            gyroTurn(.2,-3);
//            shootOne();
//            sleep(400);
//            shooter.setVelocity(0);
//            moveForwardEncoder(.5,45);

          //  strafeRight(.2,5);



            shootTop();

            sleep(1000);
            stopMotors();
            break;
        }
    }

}
