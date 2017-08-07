package org.firstinspires.ftc.hdcode.HDSamples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.hdlib.OpModeManagement.HDOpMode;
import org.firstinspires.ftc.hdlib.Sensors.AdafruitIMU;

/**
 * Created by akash on 8/4/2017.
 */

@Autonomous(name = "Sensor: BNO055 IMU Telemetry", group = "Sensor")
public class AdafruitIMUValues extends HDOpMode{
    AdafruitIMU IMU1;

    @Override
    public void initialize() {
        IMU1 = new AdafruitIMU("imu", 10);




    }

    @Override
    public void initializeLoop() {

    }

    @Override
    public void Start() {

    }

    @Override
    public void continuousRun(double elapsedTime) {
        telemetry.addData("Calibration Status", IMU1.getCalibrationStatus());
        telemetry.addData("X Heading: ", IMU1.getXheading());
        telemetry.addData("Y Heading: ", IMU1.getYheading());
        telemetry.addData("Z Heading: ", IMU1.getZheading());
        telemetry.update();

        if(gamepad1.b){
            IMU1.initializeIMU();

            while(gamepad1.b){
                idle();
            }
        }
    }


}
