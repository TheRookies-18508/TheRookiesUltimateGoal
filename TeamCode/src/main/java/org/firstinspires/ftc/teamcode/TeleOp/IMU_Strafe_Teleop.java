package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMapTheRookies;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp
public class IMU_Strafe_Teleop extends HardwareMapTheRookies{

    public double vertical;
    public double horizspeed;
    public double horiz1;
    public double horiz2;
    public double horiz3;
    public double horiz4;
    public double pivot;
    BNO055IMU imu;

    @Override
    public void runOpMode() {

        telemetry.addData("Status: ", "Hit [Init] to Initialize the bot");
        telemetry.update();

        init(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        //get and initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);


        waitForStart();
        telemetry.addData("Status: ", "OpMode Active");
        telemetry.update();
        while(opModeIsActive()){
            Orientation angles;
            double rightstickY = -gamepad1.right_stick_y;
            double leftstickY = -gamepad1.left_stick_y;
            double rightstickX = -gamepad1.right_stick_x;
            double leftstickX = -gamepad1.left_stick_x;
            double twoleftstickY = gamepad2.left_stick_y;
            double twoleftstickX = -gamepad2.left_stick_x;
            boolean rightbumper = gamepad1.right_bumper;
            boolean leftbumper = gamepad1.left_bumper;
            double error;
            double k = .09;


            vertical = -gamepad1.left_stick_y;
            horizspeed = gamepad1.right_stick_x; //strafing
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double angle = angles.firstAngle;
            if (horizspeed > 0 || horizspeed < 0) {
                double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

                while (horizspeed > 0 || horizspeed < 0) {
                    error = (startAngle - angle) * -1;
                    if (horizspeed > .2) {
                        horiz1 = ((horizspeed + (error * k)));
                        horiz2 = ((horizspeed + (error * k)));
                        horiz3 = ((horizspeed - (error * k)));
                        horiz4 = ((horizspeed - (error * k)));
                        telemetry.addData("error: ", error);
                    } else if (horizspeed < .2) {
                        horiz1 = ((horizspeed + (error * k)));
                        horiz2 = ((horizspeed + (error * k)));
                        horiz3 = ((horizspeed - (error * k)));
                        horiz4 = ((horizspeed - (error * k)));
                        telemetry.addData("error: ", error);

                    } else {
                        horiz1 = 0;
                        horiz2 = 0;
                        horiz3 = 0;
                        horiz4 = 0;
                    }
                    pivot = (-gamepad1.left_stick_x);
                    vertical = -gamepad1.left_stick_y;

                    topright.setPower((-pivot + (vertical - (horiz1))));
                    bottomright.setPower((-pivot + vertical + (horiz2)) / 2);
                    topleft.setPower((pivot + vertical + horiz3));
                    bottomleft.setPower((pivot + (vertical - (horiz4))) / 2);

                    telemetry.addData("hortiz1", horiz1);
                    telemetry.addData("hortiz2", horiz2);
                    telemetry.addData("hortiz3", horiz3);
                    telemetry.addData("hortiz4", horiz4);
                    telemetry.update();

                    if (horizspeed == 0){
                        break;
                    }

                }
            }

            telemetry.addData("hortiz1", horiz1);
            telemetry.addData("hortiz2", horiz2);
            telemetry.addData("hortiz3", horiz3);
            telemetry.addData("hortiz4", horiz4);
            telemetry.update();

            pivot = (-gamepad1.left_stick_x);

            topright.setPower((-pivot + (vertical - (horiz1))));
            bottomright.setPower((-pivot + vertical + (horiz2))/2);
            topleft.setPower((pivot + vertical + horiz3));
            bottomleft.setPower((pivot + (vertical - (horiz4)))/2);


            if (gamepad1.a){
                strafeRight(.3,3);
            }
        }
    }

}
