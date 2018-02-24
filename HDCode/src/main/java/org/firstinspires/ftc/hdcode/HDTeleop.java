package org.firstinspires.ftc.hdcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.hdlib.Controls.HDGamepad;
import org.firstinspires.ftc.hdlib.OpModeManagement.HDOpMode;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.HDRobot;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDDriveHandler;

/**
 * Created by akash on 8/8/2017.
 */

@TeleOp
public class HDTeleop extends HDOpMode implements HDGamepad.HDButtonMonitor{

    private HDGamepad driverGamepad;
    private HDGamepad servoBoyGamepad;
    private HDRobot robot;
    private double lastSpeed = 0.0;

    private enum driveMode{
        FIELD_CENTRIC_DRIVE,
        HALO_DRIVE,
        TANK_DRIVE;

        public driveMode getNext() {
            return this.ordinal() < driveMode.values().length - 1
                    ? driveMode.values()[this.ordinal() + 1]
                    : driveMode.values()[0];
        }
    }

    private double speed = 0.75;
    private driveMode curDriveMode = driveMode.HALO_DRIVE;

    @Override
    public void initialize() {

        telemetry.setMsTransmissionInterval(50);

        robot = new HDRobot(hardwareMap);

        driverGamepad = new HDGamepad(gamepad1, this);
        servoBoyGamepad = new HDGamepad(gamepad2, this);

        robot.robotDrive.reverseSide(HDDriveHandler.Side.Right);
        robot.robotDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    @Override
    public void initializeLoop() {

    }

    @Override
    public void Start() {
        driverGamepad.setGamepad(gamepad1);
        servoBoyGamepad.setGamepad(gamepad2);

    }


    @Override
    public void continuousRun(double elapsedTime) {
        if(robot.IMU1.isCalibrated()) {
            telemetry();
            driveTrain();
        }else{
            robot.robotDrive.motorBreak();
        }
    }

    private void telemetry(){
        dashboard.addProgramSpecificTelemetry(1, "Speed: " + String.valueOf(speed));
        dashboard.addProgramSpecificTelemetry(2, "Drive Mode: %s", String.valueOf(curDriveMode));
    }

    private void driveTrain(){
        if(gamepad1.a){
            robot.robotDrive.gyroTurn(90.0, 0.025, 0.000004, 0.0006, 0.0, 2.0, 0.5, -0.5, robot.IMU1.getZheading());
        }else if(gamepad1.b){
                    robot.robotDrive.gyroTurn(-90.0, 0.025, 0.000004, 0.0006, 0.0, 2.0, 0.5, -0.5, robot.IMU1.getZheading());
            }else if(gamepad1.y){
                robot.robotDrive.gyroTurn(0, 0.025, 0.000004, 0.0006, 0.0, 2.0, 0.5, -0.5, robot.IMU1.getZheading());
            }else {
                switch (curDriveMode) {
                    case FIELD_CENTRIC_DRIVE:
                        double gyroHeading = robot.IMU1.getZheading();
                    dashboard.addDiagnosticSpecificTelemetry(0, "Gyro Z Heading: %f",gyroHeading);
                    robot.robotDrive.mecanumDrive_Cartesian(gamepad1.left_stick_x * speed, gamepad1.left_stick_y * speed, gamepad1.right_stick_x * speed, gyroHeading);
                    break;
                case HALO_DRIVE:
                    robot.robotDrive.haloDrive(gamepad1.left_stick_x * speed, gamepad1.left_stick_y * speed, gamepad1.right_stick_x * speed);
                    break;
                case TANK_DRIVE:
                    robot.robotDrive.tankDrive(gamepad1.left_stick_y * speed, gamepad1.right_stick_y * speed);
                    break;
            }
        }
    }


    @Override
    public void buttonChange(HDGamepad instance, HDGamepad.gamepadButtonChange button, boolean pressed) {
        if(instance == driverGamepad){
            switch (button) {
                case A:
                    break;
                case B:
                    break;
                case X:
                    break;
                case Y:
                    break;
                case DPAD_LEFT:
                    if(pressed){
                        curDriveMode = curDriveMode.getNext();
                    }
                    break;
                case DPAD_RIGHT:
                    break;
                case DPAD_UP:
                    if(pressed) {
                        speed += 0.25;
                        speed = Range.clip(speed, 0.25, 1.0);
                    }
                    break;
                case DPAD_DOWN:
                    if(pressed) {
                        speed -= 0.25;
                        speed = Range.clip(speed, 0.25, 1.0);
                    }
                    break;
                case LEFT_BUMPER:
                    break;
                case RIGHT_BUMPER:
                    break;
                case RIGHT_TRIGGER:
                    if(pressed){
                        lastSpeed = speed;
                        speed = .5;
                    }else{
                        speed = lastSpeed;
                    }
                    break;
                case LEFT_TRIGGER:
                    break;
                case START:
                    if(pressed){
                        robot.IMU1.zeroZheading();
                    }
                    break;
            }
        }else if(instance == servoBoyGamepad){
            switch (button) {
                case A:
                    break;
                case B:
                    break;
                case X:
                    break;
                case Y:
                    break;
                case DPAD_LEFT:
                    break;
                case DPAD_RIGHT:
                    break;
                case DPAD_UP:
                    break;
                case DPAD_DOWN:
                case LEFT_BUMPER:
                    break;
                case RIGHT_BUMPER:
                    break;
                case RIGHT_TRIGGER:
                    break;
                case LEFT_TRIGGER:
                    break;
                case START:
                    break;
            }
        }
    }
}
