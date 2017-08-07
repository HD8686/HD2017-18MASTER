package org.firstinspires.ftc.hdcode.HDSamples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.hdlib.OpModeManagement.HDOpMode;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDDriveHandler;
import org.firstinspires.ftc.hdlib.Sensors.AdafruitIMU;

/**
 * Created by Height Differential on 8/7/2017.
 */

@Autonomous(name = "Gyro Turn Test")
public class gyroTurn_Test extends HDOpMode{

    HDDriveHandler robotDrive;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    AdafruitIMU IMU1;


    @Override
    public void initialize() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        IMU1 = new AdafruitIMU("imu", 10);

        robotDrive = new HDDriveHandler(frontLeft, backLeft, frontRight, backRight, true, -180, 180);
        robotDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void initializeLoop() {

    }

    @Override
    public void Start() {
        IMU1.initializeIMU();
    }

    @Override
    public void continuousRun(double elapsedTime) {
        if(IMU1.isCalibrated()) {
                robotDrive.gyroTurn(90.0, 0.009, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, IMU1.getZheading());
                telemetry.addData("3. PID_Values: ", robotDrive.getCurrentPIDResult());
                telemetry.addData("2. Gyro Z Heading: ", IMU1.getZheading());
                telemetry.addData("1. Current Error: ", robotDrive.getCurrentError());
                telemetry.addData("4. Left Motor Power: ", frontLeft.getPower());
                telemetry.addData("5. Right Motor Power: ", frontRight.getPower());
                telemetry.update();
        }else{
            robotDrive.motorBreak();
        }
    }
}
