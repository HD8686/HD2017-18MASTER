package org.firstinspires.ftc.hdcode.HDSamples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.hdlib.OpModeManagement.HDOpMode;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDDriveHandler;
import org.firstinspires.ftc.hdlib.Sensors.AdafruitIMU;

/**
 * Created by Height Differential on 8/7/2017.
 */

public class gyroTurn_Test extends HDOpMode{

    HDDriveHandler robotDrive;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    Servo tailHook;
    AdafruitIMU IMU1;


    @Override
    public void initialize() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        IMU1 = new AdafruitIMU("imu", 10);
        tailHook = hardwareMap.servo.get("tailHook");
        robotDrive = new HDDriveHandler(frontLeft, backLeft, frontRight, backRight,tailHook, true, -180, 180);
        robotDrive.reverseSide(HDDriveHandler.Side.Right);
        robotDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotDrive.setBrakeMode(DcMotor.ZeroPowerBehavior.BRAKE);
        IMU1.initializeIMU();
    }

    @Override
    public void initializeLoop() {
        dashboard.addProgramSpecificTelemetry(1, "Gyro Z Heading: %f", IMU1.getZheading());
    }

    @Override
    public void Start() {
        IMU1.initializeIMU();
    }

    @Override
    public void continuousRun(double elapsedTime) {
        dashboard.addProgramSpecificTelemetry(5, "onTarget? " + String.valueOf(robotDrive.isOnTarget()));
        if(IMU1.isCalibrated()) {
                robotDrive.gyroTurn(90.0, 0.007, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, IMU1.getZheading());
                dashboard.addProgramSpecificTelemetry(0, "PID Values: %f", robotDrive.getCurrentPIDResult());
                dashboard.addProgramSpecificTelemetry(1, "Gyro Z Heading: %f", IMU1.getZheading());
                dashboard.addProgramSpecificTelemetry(2, "Current Error: %f", robotDrive.getCurrentError());
                dashboard.addProgramSpecificTelemetry(3, "Left Motor Power: %f", frontLeft.getPower());
                dashboard.addProgramSpecificTelemetry(4, "Right Motor Power: %f", frontRight.getPower());
                dashboard.addProgramSpecificTelemetry(6, "backLeft Encoder: %d", backLeft.getCurrentPosition());
                dashboard.addProgramSpecificTelemetry(7, "frontLeft Encoder: %d", frontLeft.getCurrentPosition());
                dashboard.addProgramSpecificTelemetry(8, "backRight Encoder: %d", backRight.getCurrentPosition());
                dashboard.addProgramSpecificTelemetry(9, "frontRight Encoder: %d", frontRight.getCurrentPosition());

        }else{
            robotDrive.motorBreak();
        }
    }
}
