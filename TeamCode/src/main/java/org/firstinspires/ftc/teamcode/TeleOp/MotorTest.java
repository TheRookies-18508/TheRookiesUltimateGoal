package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMapTheRookies;

@TeleOp
public class MotorTest extends HardwareMapTheRookies {

    @Override
    public void runOpMode() {
        init(hardwareMap);

        while (opModeIsActive()) {
            if (gamepad1.a){
                topright.setPower(.4);
                telemetry.addData("Motor Running", "topright");
                telemetry.addData("Port", topright.getPortNumber());
            }
            if (gamepad1.x){
                topleft.setPower(.4);
                telemetry.addData("Motor Running", "topleft");
                telemetry.addData("Port", topleft.getPortNumber());
            }
            if (gamepad1.b){
                bottomleft.setPower(.4);
                telemetry.addData("Motor Running", "bottomleft");
                telemetry.addData("Port", bottomleft.getPortNumber());
            }
            if(gamepad1.y){
                bottomright.setPower(.4);
                telemetry.addData("Motor Running", "bottomright");
                telemetry.addData("Port", bottomright.getPortNumber());
            }
            telemetry.update();



            if (gamepad1.dpad_up){
                stopMotors();
            }
        }

    }


}
