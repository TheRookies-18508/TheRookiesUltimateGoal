package org.firstinspires.ftc.teamcode.TeleOp;


import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;


@Config
@TeleOp
public class Color_Sensor_Test extends LinearOpMode {

    public static double distance;

    private ColorSensor sensor_color_REV_ColorRangeSensor;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        sensor_color_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        // This op mode demonstrates the color and distance features of the REV sensor.
        telemetry.addData("Color Distance Example", "Press start to continue...");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Display distance info.

                distance = ((DistanceSensor) sensor_color_REV_ColorRangeSensor).getDistance(DistanceUnit.CM);

                telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


                telemetry.addData("Dist to tgt (cm)", distance);


                telemetry.update();


            }
        }

    }
}
