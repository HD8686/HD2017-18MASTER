package org.firstinspires.ftc.hdcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.hdlib.Controls.HDGamepad;
import org.firstinspires.ftc.hdlib.OpModeManagement.HDOpMode;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.HDRobot;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDDriveHandler;
import org.firstinspires.ftc.hdlib.Sensors.AdafruitIMU;

/**
 * Created by akash on 8/8/2017.
 */

@TeleOp
public class HDTeleop extends HDOpMode implements HDGamepad.HDButtonMonitor{

    HDGamepad driverGamepad;
    HDGamepad servoBoyGamepad;
    HDRobot robot;


    @Override
    public void initialize() {

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
            dashboard.addProgramSpecificTelemetry(0, "Gyro Z Heading: %f", robot.IMU1.getZheading());
            robot.robotDrive.mecanumDrive_Cartesian(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 0);
        }else{
            robot.robotDrive.motorBreak();
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
                    break;
                case DPAD_RIGHT:
                    break;
                case DPAD_UP:
                    break;
                case DPAD_DOWN:
                    break;
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
                    break;
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
