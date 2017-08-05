package org.firstinspires.ftc.hdcode.HDSamples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.hdlib.Sensors.AdafruitIMU;

/**
 * Created by akash on 8/4/2017.
 */

public class TestAdafruit extends OpMode{

    AdafruitIMU IMU1;

    @Override
    public void init() {
        IMU1 = new AdafruitIMU("imu", 10);
    }

    @Override
    public void loop() {
        telemetry.addData("Calibration Status", IMU1.getCalibrationStatus());
        telemetry.addData("X Heading: ", IMU1.getXheading());
        telemetry.addData("Y Heading: ", IMU1.getYheading());
        telemetry.addData("Z Heading: ", IMU1.getZheading());
    }


}
